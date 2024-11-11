#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <derived_object_msgs/ObjectArray.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <chrono>
#include <map>
#include <set>
#include <Eigen/Dense>
#include <cmath>
#include <opencv2/opencv.hpp>  // For Kalman Filter
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <lidar_processing/Tracking.h>
#include <lidar_processing/TrackingArray.h>


ros::Publisher downsampled_pub;
ros::Publisher ransac_pub;
ros::Publisher road_pub;
ros::Publisher cluster_marker_pub;
ros::Publisher tracking_marker_pub;  // Publisher for tracking markers
ros::Publisher cluster_info_pub;     // Publisher for cluster information
ros::Publisher marker_pub;
ros::Publisher tracking_pub;

std::map<int, cv::KalmanFilter> kalman_filters;
std::map<int, cv::Mat> measurements;
std::map<int, ros::Time> cluster_creation_times;
std::map<int, Eigen::Vector3f> previous_cluster_centers; // Previous frame cluster centers
Eigen::Vector3f vehicle_position(0.0, 0.0, 0.0);  // Initialize vehicle position
float lidar_height = 2.25;  // Height of the LIDAR sensor above the vehicle's origin
Eigen::Vector3f imu_rotation_vector(0.0f, 0.0f, 0.0f);

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // 오일러 각도를 추출합니다.
    Eigen::Quaternionf quat(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    Eigen::Vector3f euler_angles = quat.toRotationMatrix().eulerAngles(0, 1, 2);

    // 오일러 각도 (롤, 피치, 요)를 전역 변수에 저장합니다.
    imu_rotation_vector = euler_angles;

    // 필요한 경우, 여기서 회전 행렬을 미리 계산해둘 수 있습니다.
}

void objectCallback(const derived_object_msgs::ObjectArray::ConstPtr& msg) {
    visualization_msgs::MarkerArray marker_array;

    for (const auto& object : msg->objects) {
        if (object.classification == 6) {  // Check if the object is a car
            // Extract pose
            const auto& pose = object.pose.position;
            const auto& orientation = object.pose.orientation;
            
            // Extract speed and yaw
            float vx = object.twist.linear.x;
            float vy = object.twist.linear.y;
            float speed = std::sqrt(vx * vx + vy * vy) * 3.6;  // Convert m/s to km/h
            

            Eigen::Matrix3f rotation;
            rotation = Eigen::AngleAxisf(imu_rotation_vector.z(), Eigen::Vector3f::UnitZ()) * // 요
                       Eigen::AngleAxisf(imu_rotation_vector.y(), Eigen::Vector3f::UnitY()) * // 피치
                       Eigen::AngleAxisf(imu_rotation_vector.x(), Eigen::Vector3f::UnitX());  // 롤

            // 회전 행렬의 역행렬을 계산합니다.
            Eigen::Matrix3f rotation_inv = rotation.inverse();
            Eigen::Vector3f speed_vector_3d(vx, vy, 0);  // Z 축 방향 속도는 0으로 가정
            Eigen::Vector3f transformed_speed_vector = rotation_inv * speed_vector_3d;

            // 변환된 속도 벡터로부터 yaw를 계산합니다.
            float transformed_vx = transformed_speed_vector.x();
            float transformed_vy = transformed_speed_vector.y();
            float yaw = std::atan2(transformed_vy, transformed_vx);

            // 차량 위치와 LiDAR 높이를 사용하여 변환 벡터를 정의합니다.
            Eigen::Vector3f transform(vehicle_position.x(), vehicle_position.y(), vehicle_position.z() + lidar_height);

            // 절대 좌표를 Eigen::Vector3f로 변환합니다.
            Eigen::Vector3f absolute_position(pose.x, pose.y, pose.z);

            // 절대 좌표를 차량 좌표계로 변환합니다.
            Eigen::Vector3f relative_position = rotation_inv * (absolute_position - transform);

            // Bounding Box
            visualization_msgs::Marker box_marker;
            box_marker.header.frame_id = "hero/LIDAR";
            box_marker.header.stamp = ros::Time::now();
            box_marker.ns = "objects";
            box_marker.id = object.id;
            box_marker.type = visualization_msgs::Marker::CUBE;
            box_marker.action = visualization_msgs::Marker::ADD;
            box_marker.pose.position.x = relative_position.x();
            box_marker.pose.position.y = relative_position.y();
            box_marker.pose.position.z = relative_position.z();
            // box_marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            box_marker.scale.x = object.shape.dimensions[0];
            box_marker.scale.y = object.shape.dimensions[1];
            box_marker.scale.z = object.shape.dimensions[2];  // Line width
            box_marker.color.r = 0.0;
            box_marker.color.g = 1.0;
            box_marker.color.b = 0.0;
            box_marker.color.a = 0.0;
            


            box_marker.lifetime = ros::Duration(0.05);

            marker_array.markers.push_back(box_marker);

            // Speed Marker
            visualization_msgs::Marker speed_marker;
            speed_marker.header.frame_id = "hero/LIDAR";
            speed_marker.header.stamp = ros::Time::now();
            speed_marker.ns = "objects";
            speed_marker.id = object.id + 10000;  // Ensure unique ID for text markers
            speed_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            speed_marker.action = visualization_msgs::Marker::ADD;
            speed_marker.pose.position.x = relative_position.x();
            speed_marker.pose.position.y = relative_position.y();
            speed_marker.pose.position.z = relative_position.z()-5;
            speed_marker.scale.z = 1.0;
            speed_marker.color.r = 1.0;
            speed_marker.color.g = 1.0;
            speed_marker.color.b = 1.0;
            speed_marker.color.a = 1.0;
            speed_marker.text = "Speed: " + std::to_string(speed) + " km/h";
            speed_marker.lifetime = ros::Duration(0.05);

            marker_array.markers.push_back(speed_marker);

            // Yaw Marker
            visualization_msgs::Marker yaw_marker;
            yaw_marker.header.frame_id = "hero/LIDAR";
            yaw_marker.header.stamp = ros::Time::now();
            yaw_marker.ns = "objects";
            yaw_marker.id = object.id + 20000;  // Ensure unique ID for yaw markers
            yaw_marker.type = visualization_msgs::Marker::ARROW;
            yaw_marker.action = visualization_msgs::Marker::ADD;
            yaw_marker.pose.position.x = relative_position.x();
            yaw_marker.pose.position.y = relative_position.y();
            yaw_marker.pose.position.z = relative_position.z();
            yaw_marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            yaw_marker.scale.x = 1.0;  // Arrow length
            yaw_marker.scale.y = 1.0;  // Arrow head width
            yaw_marker.scale.z = 0.1;  // Arrow shaft width
            yaw_marker.color.r = 0.0;
            yaw_marker.color.g = 0.0;
            yaw_marker.color.b = 1.0;
            yaw_marker.color.a = 1.0;
            yaw_marker.lifetime = ros::Duration(0.05);

            marker_array.markers.push_back(yaw_marker);
        }
    }

    marker_pub.publish(marker_array);
}
int generateNewID() {
    static int current_id = 0;
    return current_id++;
}
void position_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    vehicle_position = Eigen::Vector3f(msg->x, msg->y, msg->z);
}

void downsampling_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    auto t0 = std::chrono::high_resolution_clock::now();

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    if (cloud->empty()) {
        ROS_WARN("Received an empty point cloud.");
        return;
    }
    ROS_INFO("Points before downsampling: %zu", cloud->points.size());

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.2f, 0.2f, 0.2f);
    voxel_grid.filter(*downsampled_cloud);

    ROS_INFO("Points after downsampling: %zu", downsampled_cloud->points.size());

    if (downsampled_cloud->empty()) {
        ROS_WARN("Downsampled point cloud is empty.");
        return;
    }

    pcl::PCLPointCloud2 downsampled_pcl_pc2;
    pcl::toPCLPointCloud2(*downsampled_cloud, downsampled_pcl_pc2);
    sensor_msgs::PointCloud2 downsampled_msg;
    pcl_conversions::fromPCL(downsampled_pcl_pc2, downsampled_msg);
    downsampled_msg.header = msg->header;
    downsampled_pub.publish(downsampled_msg);

    pcl::SACSegmentation<pcl::PointXYZ> sac_segmentation;
    sac_segmentation.setInputCloud(downsampled_cloud);
    sac_segmentation.setMethodType(pcl::SAC_RANSAC);
    sac_segmentation.setModelType(pcl::SACMODEL_PLANE);
    sac_segmentation.setDistanceThreshold(0.5);
    sac_segmentation.setMaxIterations(3000);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    sac_segmentation.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        ROS_WARN("No inliers found for the plane model.");
        return;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr road_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    extract.setInputCloud(downsampled_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*road_cloud);

    extract.setNegative(true);
    extract.filter(*obstacle_cloud);

    ROS_INFO("Points after RANSAC: Road points %zu, Obstacle points %zu", road_cloud->points.size(), obstacle_cloud->points.size());

    pcl::PCLPointCloud2 ransac_pcl_pc2;
    pcl::toPCLPointCloud2(*obstacle_cloud, ransac_pcl_pc2);
    sensor_msgs::PointCloud2 ransac_msg;
    pcl_conversions::fromPCL(ransac_pcl_pc2, ransac_msg);
    ransac_msg.header = msg->header;
    ransac_pub.publish(ransac_msg);

    pcl::PCLPointCloud2 road_pcl_pc2;
    pcl::toPCLPointCloud2(*road_cloud, road_pcl_pc2);
    sensor_msgs::PointCloud2 road_msg;
    pcl_conversions::fromPCL(road_pcl_pc2, road_msg);
    road_msg.header = msg->header;
    road_pub.publish(road_msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr non_vehicle_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    float exclude_x_min = -3.0, exclude_x_max = -1.0;
    float exclude_y_min = -1.0, exclude_y_max = 1.0;

    for (const auto& point : obstacle_cloud->points) {
        if (!((point.x >= exclude_x_min && point.x <= exclude_x_max) &&
              (point.y >= exclude_y_min && point.y <= exclude_y_max))) {
            non_vehicle_cloud->points.push_back(point);
        }
    }

    ROS_INFO("Points after excluding specific range: %zu", non_vehicle_cloud->points.size());

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setInputCloud(non_vehicle_cloud);
    ec.setClusterTolerance(1.0);
    ec.setMinClusterSize(0);
    ec.setMaxClusterSize(3000);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::MarkerArray tracking_marker_array;
    visualization_msgs::MarkerArray info_marker_array; // New marker array for info
    ros::Time now = ros::Time::now();
    std::map<int, Eigen::Vector3f> current_cluster_centers; // Current frame cluster centers
    std::set<int> active_cluster_ids;
    lidar_processing::TrackingArray tracking_array_msg;
    lidar_processing::Tracking tracking_msg;

    for (const auto& indices : cluster_indices) {
        Eigen::Vector4f cluster_min_pt, cluster_max_pt;
        pcl::getMinMax3D(*non_vehicle_cloud, indices.indices, cluster_min_pt, cluster_max_pt);

        if (cluster_max_pt[2] > 0.1f) {
            continue;
        }

        Eigen::Vector3f cluster_size = cluster_max_pt.head<3>() - cluster_min_pt.head<3>();
        Eigen::Vector3f min_dimensions(0.1f, 0.1f, 0.1f);
        Eigen::Vector3f max_dimensions(100.0f, 100.0f, 2.0f);

        if ((cluster_size.array() < min_dimensions.array()).any() || 
            (cluster_size.array() > max_dimensions.array()).any()) {
            continue;
        }
        // Correct way to initialize the Eigen::Vector3f
        Eigen::Vector3f transform(vehicle_position.x(), vehicle_position.y(), vehicle_position.z() + lidar_height);
        Eigen::Matrix3f rotation;
        rotation =  Eigen::AngleAxisf(imu_rotation_vector.z(), Eigen::Vector3f::UnitZ()) * // 요
                    Eigen::AngleAxisf(imu_rotation_vector.y(), Eigen::Vector3f::UnitY()) * // 피치
                    Eigen::AngleAxisf(imu_rotation_vector.x(), Eigen::Vector3f::UnitX());  // 롤


        // Compute the center of the cluster
        Eigen::Vector3f cluster_centera = (cluster_min_pt.head<3>() + cluster_max_pt.head<3>()) / 2.0f;

        // Apply the transformation to the cluster center
        Eigen::Vector3f cluster_center = rotation * cluster_centera + transform;


        int cluster_id = -1;
        float min_distance = std::numeric_limits<float>::max();

        // Nearest Neighbor: Find the closest previous cluster
        for (const auto& prev_cluster : previous_cluster_centers) {
            const auto& prev_id = prev_cluster.first;
            const auto& prev_center = prev_cluster.second;
            float distance = (cluster_center - prev_center).norm();

            if (distance < min_distance) {
                min_distance = distance;
                cluster_id = prev_id;
            }
        }

        // If a close cluster exists, reuse the ID; otherwise, generate a new ID
        if (min_distance < 2.0f && cluster_id != -1) {
            // Reuse the existing cluster_id
        } else {
            cluster_id = generateNewID();
        }

        active_cluster_ids.insert(cluster_id);
        current_cluster_centers[cluster_id] = cluster_center;

        // Create Kalman Filter for new clusters
        if (kalman_filters.find(cluster_id) == kalman_filters.end()) {
            cv::KalmanFilter kf(6, 3, 0); // State: [x, y, z, vx, vy, vz], Measurement: [x, y, z]
            float delta_t = 0.05f;
            kf.transitionMatrix = (cv::Mat_<float>(6, 6) <<
                1, 0, 0, delta_t, 0, 0,
                0, 1, 0, 0, delta_t, 0,
                0, 0, 1, 0, 0, delta_t,
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 1);

            kf.measurementMatrix = (cv::Mat_<float>(3, 6) <<
                1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0);

            cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-4));
            cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-2));
            cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));

            kf.statePost.at<float>(0) = cluster_center[0];
            kf.statePost.at<float>(1) = cluster_center[1];
            kf.statePost.at<float>(2) = cluster_center[2];
            kf.statePost.at<float>(3) = 0;
            kf.statePost.at<float>(4) = 0;
            kf.statePost.at<float>(5) = 0;

            kalman_filters[cluster_id] = kf;
            measurements[cluster_id] = cv::Mat_<float>(3, 1) << cluster_center[0], cluster_center[1], cluster_center[2];
            cluster_creation_times[cluster_id] = now;
        } else {
            // Update Kalman Filter with new measurement
            measurements[cluster_id].at<float>(0) = cluster_center[0];
            measurements[cluster_id].at<float>(1) = cluster_center[1];
            measurements[cluster_id].at<float>(2) = cluster_center[2];
            kalman_filters[cluster_id].correct(measurements[cluster_id]);
        }

        cv::Mat prediction = kalman_filters[cluster_id].predict();
        Eigen::Vector3f predicted_center(
            prediction.at<float>(0),
            prediction.at<float>(1),
            prediction.at<float>(2)
        );
        // Eigen::Matrix3f rotationa;
        // rotation = Eigen::AngleAxisf(imu_rotation_vector.z(), Eigen::Vector3f::UnitZ()) * // 요
        //                Eigen::AngleAxisf(imu_rotation_vector.y(), Eigen::Vector3f::UnitY()) * // 피치
        //                Eigen::AngleAxisf(imu_rotation_vector.x(), Eigen::Vector3f::UnitX());  // 롤

            // 회전 행렬의 역행렬을 계산합니다.
        Eigen::Matrix3f rotation_inv = rotation.inverse();

            // 차량 위치와 LiDAR 높이를 사용하여 변환 벡터를 정의합니다.
        // Eigen::Vector3f transform(vehicle_position.x(), vehicle_position.y(), vehicle_position.z() + lidar_height);

            // 절대 좌표를 Eigen::Vector3f로 변환합니다.
        Eigen::Vector3f cluster_position(cluster_center[0], cluster_center[1], cluster_center[2]);

            // 절대 좌표를 차량 좌표계로 변환합니다.
        Eigen::Vector3f rel_position = rotation_inv * (cluster_position - transform);

        // Create cluster marker
        visualization_msgs::Marker cluster_marker;
        cluster_marker.header.frame_id = "hero/LIDAR";
        cluster_marker.header.stamp = now;
        cluster_marker.ns = "clusters";
        cluster_marker.id = cluster_id;
        cluster_marker.type = visualization_msgs::Marker::CUBE;
        cluster_marker.action = visualization_msgs::Marker::ADD;
        cluster_marker.pose.position.x = rel_position.x();
        cluster_marker.pose.position.y = rel_position.y();
        cluster_marker.pose.position.z = rel_position.z();
        cluster_marker.scale.x = cluster_size[0];
        cluster_marker.scale.y = cluster_size[1];
        cluster_marker.scale.z = cluster_size[2];
        cluster_marker.color.r = 1.0f;
        cluster_marker.color.g = 0.0f;
        cluster_marker.color.b = 0.0f;
        cluster_marker.color.a = 0.5f;
        cluster_marker.lifetime = ros::Duration(0.05);
        marker_array.markers.push_back(cluster_marker);

        // Create text marker for ID
        visualization_msgs::Marker id_marker;
        id_marker.header.frame_id = "hero/LIDAR";
        id_marker.header.stamp = now;
        id_marker.ns = "cluster_info";
        id_marker.id = cluster_id + 10000;  // Ensure unique ID for text markers
        id_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        id_marker.action = visualization_msgs::Marker::ADD;
        id_marker.pose.position.x = rel_position.x();
        id_marker.pose.position.y = rel_position.y();
        id_marker.pose.position.z = rel_position.z();
        id_marker.scale.z = 1.0;
        id_marker.color.r = 1.0f;
        id_marker.color.g = 1.0f;
        id_marker.color.b = 1.0f;
        id_marker.color.a = 1.0f;
        id_marker.text = "ID: " + std::to_string(cluster_id);
        id_marker.lifetime = ros::Duration(0.05);
        info_marker_array.markers.push_back(id_marker);

        // Create text marker for speed (using Kalman filter's velocity estimate)
        visualization_msgs::Marker speed_marker;
        speed_marker.header.frame_id = "hero/LIDAR";
        speed_marker.header.stamp = now;
        speed_marker.ns = "cluster_info";
        speed_marker.id = cluster_id + 20000;  // Ensure unique ID for text markers
        speed_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        speed_marker.action = visualization_msgs::Marker::ADD;
        speed_marker.pose.position.x = rel_position.x();
        speed_marker.pose.position.y = rel_position.y();
        speed_marker.pose.position.z = rel_position.z()-5;
        speed_marker.scale.z = 1.0;
        speed_marker.color.r = 0.0f;
        speed_marker.color.g = 1.0f;
        speed_marker.color.b = 0.0f;
        speed_marker.color.a = 1.0f;
        float vx = prediction.at<float>(3);
        float vy = prediction.at<float>(4);
        float vz = prediction.at<float>(5);
        float speed = std::sqrt(vx*vx + vy*vy + vz*vz);
        float speed_kmh = speed * 3.6f;
        speed_marker.text = "Speed: " + std::to_string(speed_kmh) + " km/h";
        speed_marker.lifetime = ros::Duration(0.05);
        info_marker_array.markers.push_back(speed_marker);

        // Create yaw direction marker
        // Create yaw direction marker
        if (speed >1.0){
            visualization_msgs::Marker yaw_marker;
            yaw_marker.header.frame_id = "hero/LIDAR";
            yaw_marker.header.stamp = ros::Time::now();
            yaw_marker.ns = "cluster_info";
            yaw_marker.id = cluster_id + 30000;  // Ensure unique ID for arrow markers
            yaw_marker.type = visualization_msgs::Marker::ARROW;
            yaw_marker.action = visualization_msgs::Marker::ADD;
            yaw_marker.pose.position.x = rel_position.x();
            yaw_marker.pose.position.y = rel_position.y();
            yaw_marker.pose.position.z = rel_position.z();

            // Calculate the yaw angle from vx and vy

            Eigen::Vector3f speed_vector_3d(vx, vy, 0);  // Z 축 방향 속도는 0으로 가정
            float yaw_anglee = std::atan2(vy, vx);
            Eigen::Vector3f transformed_speed_vector = rotation_inv * speed_vector_3d;

            // 변환된 속도 벡터로부터 yaw를 계산합니다.
            float transformed_vx = transformed_speed_vector.x();
            float transformed_vy = transformed_speed_vector.y();
            float yaw_angle = std::atan2(transformed_vy, transformed_vx);

            // Use tf::createQuaternionMsgFromYaw to set the orientation
            yaw_marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_angle);

            yaw_marker.scale.x = 1.0; // Arrow width
            yaw_marker.scale.y = 1.0; // Arrow head width
            yaw_marker.scale.z = 0.1; // Arrow length
            yaw_marker.color.r = 0.0f;
            yaw_marker.color.g = 0.0f;
            yaw_marker.color.b = 1.0f;
            yaw_marker.color.a = 1.0f;
            yaw_marker.lifetime = ros::Duration(0.05);

            // Add marker to the marker array
            tracking_marker_array.markers.push_back(yaw_marker);
            
            tracking_msg.cluster_id = cluster_id;
            tracking_msg.x = static_cast<float_t>(cluster_center[0]);
            tracking_msg.y = static_cast<float_t>(cluster_center[1]);
            tracking_msg.vx = static_cast<float_t>(prediction.at<float>(3));
            tracking_msg.vy = static_cast<float_t>(prediction.at<float>(4));
            tracking_msg.yaw = static_cast<float_t>(yaw_anglee); // Converting radians to degrees

            // Add the tracking message to the tracking array
            tracking_array_msg.tracks.push_back(tracking_msg);


            
        }

        }
    tracking_pub.publish(tracking_array_msg);
    // Publish markers
    cluster_marker_pub.publish(marker_array);
    cluster_info_pub.publish(info_marker_array);
    tracking_marker_pub.publish(tracking_marker_array);

    // Update previous cluster centers with current centers
    previous_cluster_centers = current_cluster_centers;

    // Remove stale clusters from Kalman filters and measurements
    for (auto it = kalman_filters.begin(); it != kalman_filters.end(); ) {
        int id = it->first;
        if (active_cluster_ids.find(id) == active_cluster_ids.end()) {
            it = kalman_filters.erase(it);
            measurements.erase(id);
            cluster_creation_times.erase(id);
        } else {
            ++it;
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    ROS_INFO("Callback processing time: %f seconds", std::chrono::duration<double>(t1 - t0).count());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "downsampling_node");
    ros::NodeHandle nh;

    downsampled_pub = nh.advertise<sensor_msgs::PointCloud2>("/carla/hero/lidar_downsampled", 1);
    ransac_pub = nh.advertise<sensor_msgs::PointCloud2>("/carla/hero/lidar_ransac", 1);
    road_pub = nh.advertise<sensor_msgs::PointCloud2>("/carla/hero/lidar_road_surface_pcd", 1);
    cluster_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/carla/hero/lidar_clusters", 1);
    tracking_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/carla/hero/lidar_tracking_markers", 1);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    cluster_info_pub = nh.advertise<visualization_msgs::MarkerArray>("/carla/hero/lidar_cluster_info", 1);
    tracking_pub = nh.advertise<lidar_processing::TrackingArray>("/object_tracking", 1);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::Point>("/enu_coordinates", 10, position_callback);
    ros::Subscriber sub = nh.subscribe("/carla/hero/LIDAR", 1, downsampling_callback);
    ros::Subscriber object_sub = nh.subscribe("/carla/hero/Objects", 1, objectCallback);
    ros::Subscriber IMU_sub = nh.subscribe("/carla/hero/IMU", 1, imuCallback);
    ros::spin();
    return 0;
}
