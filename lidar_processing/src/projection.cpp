#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <your_package/DetectionInfoArray.h>
#include <your_package/DetectionInfo.h>

// 카메라 내부 파라미터
const double CAMERA_FX = 335.639852470912;
const double CAMERA_FY = 335.639852470912;
const double CAMERA_CX = 400.0;
const double CAMERA_CY = 300.0;

// 카메라와 LIDAR 간의 extrinsic 파라미터
const double LIDAR_TO_CAMERA_X = 0.7;
const double LIDAR_TO_CAMERA_Y = 0.0;
const double LIDAR_TO_CAMERA_Z = -0.65;

// Roll, Pitch, Yaw 각도를 라디안 단위로 정의
const double ROLL = 0;    // 0도
const double PITCH = 0;   // 0도
const double YAW = 0;     // 0도

class SensorFusionNode {
public:
    SensorFusionNode() {
        ros::NodeHandle nh;
        lidar_sub_ = nh.subscribe("/carla/hero/lidar_ransac", 1, &SensorFusionNode::lidarCallback, this);
        image_sub_ = nh.subscribe("/carla/hero/Center/image", 1, &SensorFusionNode::imageCallback, this);
        detection_sub_ = nh.subscribe("yolo_data", 10, &SensorFusionNode::detectionCallback, this);
        detection_pub_ = nh.advertise<your_package::DetectionInfoArray>("detection_with_distance", 10);
        image_pub_ = nh.advertise<sensor_msgs::Image>("/carla/hero/Center/image_with_lidar", 10);
        marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    }

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);

        // PCLPointCloud2를 pcl::PointCloud<pcl::PointXYZ>로 변환
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        // 변환된 포인트 클라우드를 클래스 멤버에 저장
        cloud_ = cloud;

        // Roll, Pitch, Yaw를 고려한 회전 행렬 계산
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix <<
            0, -1, 0,
            0, 0, -1,
            1, 0, 0;

        // 변환 벡터 정의
        Eigen::Vector3d translation_vector(LIDAR_TO_CAMERA_X, LIDAR_TO_CAMERA_Y, LIDAR_TO_CAMERA_Z);

        // Extrinsic 매트릭스 생성 (3x4 행렬)
        Eigen::Matrix<double, 3, 4> extrinsic_matrix;
        extrinsic_matrix.block<3,3>(0,0) = rotation_matrix;
        extrinsic_matrix.col(3) = translation_vector;

        // Intrinsic 매트릭스 (3x3 형태)
        Eigen::Matrix3d intrinsic_matrix;
        intrinsic_matrix <<
            CAMERA_FX, 0, CAMERA_CX,
            0, CAMERA_FY, CAMERA_CY,
            0, 0, 1;

        // 저장하여 이미지를 그리기 위해 나중에 사용
        extrinsic_matrix_ = extrinsic_matrix;
        intrinsic_matrix_ = intrinsic_matrix;
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        if (!cloud_ || cloud_->points.empty()) {
            ROS_WARN("Point cloud is not initialized or empty.");
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Marker 설정
        visualization_msgs::Marker marker;
        marker.header.frame_id = "hero/LIDAR"; // 적절한 프레임 ID로 설정
        marker.header.stamp = ros::Time::now();
        marker.ns = "points_and_lines";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1; // 점의 크기 (X축 방향)
        marker.scale.y = 0.1; // 점의 크기 (Y축 방향)
        marker.scale.z = 0.1;
        marker.color.r = 1.0f; // 빨간색
        marker.color.g = 0.0f; // 초록색
        marker.color.b = 0.0f; // 파란색
        marker.color.a = 1.0; // 투명도

        // Detection info array 초기화
        your_package::DetectionInfoArray detection_array;

        // 포인트 클라우드를 이미지에 투영하고 빨간색 점을 그림
        for (const auto& point : cloud_->points) {
            Eigen::Vector4d homogenous_world_point(point.x, point.y, point.z, 1.0);
            Eigen::Vector3d camera_point = extrinsic_matrix_ * homogenous_world_point;

            // 카메라 뒤쪽에 있는 포인트는 무시
            if (camera_point[2] <= 0) {
                continue;
            }

            Eigen::Vector3d image_point = intrinsic_matrix_ * camera_point;

            double u = image_point[0] / image_point[2];
            double v = image_point[1] / image_point[2];

            if (u >= 0 && u < cv_ptr->image.cols && v >= 0 && v < cv_ptr->image.rows) {
                cv::circle(cv_ptr->image, cv::Point(u, v), 1, cv::Scalar(0, 0, 255), -1); // 빨간색 점

                geometry_msgs::Point p;
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;
                marker.points.push_back(p);

                // Detection box와 비교하여 거리 정보를 업데이트
                for (const auto& detection : detections_.detections) {
                    double x_min = detection.center_x - detection.width / 2;
                    double x_max = detection.center_x + detection.width / 2;
                    double y_min = detection.center_y - detection.height / 2;
                    double y_max = detection.center_y + detection.height / 2;

                    if (u >= x_min && u <= x_max && v >= y_min && v <= y_max) {
                        your_package::DetectionInfo detection_with_distance = detection;
                        detection_with_distance.distance = sqrt(point.x * point.x + point.y * point.y);
                        detection_array.detections.push_back(detection_with_distance);
                        break; // 하나의 detection box에서만 거리 정보를 추가하므로 break
                    }
                }
            }
        }

        // Marker 퍼블리시
        marker_pub_.publish(marker);

        // Detection array 퍼블리시
        detection_pub_.publish(detection_array);

        // 이미지 퍼블리시
        image_pub_.publish(cv_ptr->toImageMsg());
    }

    void detectionCallback(const your_package::DetectionInfoArray::ConstPtr& msg) {
        detections_ = *msg;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber image_sub_;
    ros::Subscriber detection_sub_;
    ros::Publisher image_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher detection_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    Eigen::Matrix<double, 3, 4> extrinsic_matrix_;
    Eigen::Matrix3d intrinsic_matrix_;
    your_package::DetectionInfoArray detections_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_fusion_node");
    SensorFusionNode node;
    ros::spin();
    return 0;
}
