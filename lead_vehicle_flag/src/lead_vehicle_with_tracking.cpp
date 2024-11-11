#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <std_msgs/Int32.h>
#include "lead_vehicle_flag/Tracking.h"
#include "lead_vehicle_flag/TrackingArray.h"

class CarlaListener {
public:
    CarlaListener() : nh_("~"), ego_yaw_(0.0), ego_pitch_(0.0), ego_roll_(0.0) {
        lead_vehicle_status_pub_ = nh_.advertise<geometry_msgs::Twist>("/carla/lead_vehicle_status", 10);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/carla/vehicle_markers", 10);

        // Subscribers
        position_sub_ = nh_.subscribe("/enu_coordinates", 10, &CarlaListener::enuCoordinatesCallback, this);  // 변경된 부분
        current_index_sub_ = nh_.subscribe("/current_point_index", 10, &CarlaListener::currentIndexCallback, this);
        tracking_sub_ = nh_.subscribe("/object_tracking", 10, &CarlaListener::trackingCallback, this);

        loadRefPathFromCSV("/home/ailab/catkin_ws/src/lead_vehicle_flag/src/smoothing_path.csv");
    }

    void run() {
        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher lead_vehicle_status_pub_;
    ros::Publisher marker_pub_;
    ros::Subscriber position_sub_, current_index_sub_, tracking_sub_;

    int current_index_;
    double ego_yaw_, ego_pitch_, ego_roll_;
    geometry_msgs::Point enu_coordinates_;

    std::vector<geometry_msgs::Point> ref_path_;

    double calculateDistance(const geometry_msgs::Point& pos1, const geometry_msgs::Point& pos2) {
        return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2));
    }

    double angleMod(double angle) {
        return fmod(angle + M_PI, 2 * M_PI) - M_PI;
    }

    void loadRefPathFromCSV(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open file: %s", filename.c_str());
            return;
        }
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            geometry_msgs::Point point;
            std::string value;

            std::getline(ss, value, ',');
            point.x = std::stod(value);
            std::getline(ss, value, ',');
            point.y = std::stod(value);
            point.z = 0.0;

            ref_path_.push_back(point);
        }

        file.close();
        ROS_INFO("Loaded %lu waypoints from %s", ref_path_.size(), filename.c_str());
    }

    void enuCoordinatesCallback(const geometry_msgs::Point::ConstPtr& data) {
        enu_coordinates_ = *data;
        ROS_INFO("ENU coordinates received: x=%.2f, y=%.2f, z=%.2f", enu_coordinates_.x, enu_coordinates_.y, enu_coordinates_.z);
    }
    void currentIndexCallback(const std_msgs::Int32::ConstPtr& msg) {
        current_index_ = msg->data;
    }

    void trackingCallback(const lead_vehicle_flag::TrackingArray::ConstPtr& data) {
        std::vector<const lead_vehicle_flag::Tracking*> closest_tracks(2, nullptr);
        std::vector<double> min_distances(2, std::numeric_limits<double>::infinity());
        visualization_msgs::MarkerArray marker_array;

        for (const auto& track : data->tracks) {
            // 자차 위치에서 object 위치를 뺀 상대 위치 계산
            Eigen::Vector3d relative_position(track.x - enu_coordinates_.x, 
                                              track.y - enu_coordinates_.y, 
                                              0.0);  // z 값은 필요 없다면 0으로 설정

            // 회전 행렬 생성
            Eigen::Matrix3d rotation_matrix;
            rotation_matrix = Eigen::AngleAxisd(ego_yaw_, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(ego_pitch_, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(ego_roll_, Eigen::Vector3d::UnitX());

            // 회전 행렬을 이용해 상대 위치 변환
            Eigen::Vector3d transformed_position = rotation_matrix * relative_position;

            // Check if the object is in front of the ego vehicle
            if (transformed_position.x() < 0) {
                continue;
            }

            double distance = transformed_position.norm();  // 벡터의 길이로 거리 계산

            // Filter out objects too close to the ego vehicle
            if (distance < 1.0) {
                continue;
            }

            // Filter out objects that are not within a certain lateral range
            if (std::abs(transformed_position.y()) > 1.25) {
                continue;
            }

            // Filter based on yaw (assuming cluster_id indicates the lead vehicle)
            double valid_yaw_threshold = 0.174;  // Approx ±10 degrees
            bool valid_angle = std::abs(angleMod(track.yaw - ego_yaw_)) < valid_yaw_threshold;

            // Create a marker for visualization
            visualization_msgs::Marker marker;
            marker.header.frame_id = "hero";
            marker.header.stamp = ros::Time::now();
            marker.ns = "tracks";
            marker.id = track.cluster_id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            // 변환된 상대 위치를 마커의 위치로 설정
            marker.pose.position.x = transformed_position.x();
            marker.pose.position.y = transformed_position.y();
            marker.pose.position.z = transformed_position.z();
            marker.pose.orientation = tf::createQuaternionMsgFromYaw(track.yaw);

            marker.scale.x = 4.5;  // Example scale, adjust based on your needs
            marker.scale.y = 2.0;
            marker.scale.z = 1.5;

            marker.color.a = 0.5;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;  // Default color blue
            marker.lifetime = ros::Duration(1.0);
            marker_array.markers.push_back(marker);

            ROS_INFO("Marker ID: %d, Position: x=%.2f, y=%.2f, z=%.2f, Yaw: %.2f", 
                      marker.id, marker.pose.position.x, marker.pose.position.y, marker.pose.position.z, track.yaw);

            if (valid_angle && distance <= 30.0) {  
                if (distance < min_distances[0]) {
                    min_distances[1] = min_distances[0];
                    closest_tracks[1] = closest_tracks[0];
                    min_distances[0] = distance;
                    closest_tracks[0] = &track;
                } else if (distance < min_distances[1]) {
                    min_distances[1] = distance;
                    closest_tracks[1] = &track;
                }
            }
        }

        // Highlight the lead vehicle(s) in red
        for (const auto& lead : closest_tracks) {
            if (lead == nullptr) continue;

            for (auto& marker : marker_array.markers) {
                if (marker.id == lead->cluster_id) {
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;  // Change to red
                    marker.color.a = 1.0;  // Make it opaque
                }
            }
        }
        
        marker_pub_.publish(marker_array);

        if (closest_tracks[0] != nullptr) {
            ROS_INFO("Lead Vehicle 1 ID: %d", static_cast<int>(closest_tracks[0]->cluster_id));
            if (closest_tracks[1] != nullptr) {
                ROS_INFO("Lead Vehicle 2 ID: %d", static_cast<int>(closest_tracks[1]->cluster_id));
            }

            for (size_t i = 0; i < closest_tracks.size(); ++i) {
                if (closest_tracks[i] == nullptr) continue;

                const auto& track = *closest_tracks[i];

                // Create a geometry_msgs::Point for the distance calculation
                geometry_msgs::Point track_point;
                track_point.x = track.x;
                track_point.y = track.y;
                track_point.z = 0.0;

                // Calculate distance to the lead vehicle
                double distance_to_lead = calculateDistance(enu_coordinates_, track_point);

                // Publish the lead vehicle status
                geometry_msgs::Twist status_msg;
                status_msg.linear.x = track.x;
                status_msg.linear.y = track.y;
                status_msg.linear.z = std::sqrt(std::pow(track.vx, 2) + std::pow(track.vy, 2));  // Speed
                status_msg.angular.z = track.yaw;

                ROS_INFO("Lead Vehicle %d Position: x=%.2f, y=%.2f, Speed: %.2f, Yaw: %.2f", 
                         static_cast<int>(i + 1), track.x, track.y, status_msg.linear.z, track.yaw);
                ROS_INFO("Distance to Lead Vehicle %d: %.2f meters", static_cast<int>(i + 1), distance_to_lead);

                lead_vehicle_status_pub_.publish(status_msg);
            }
        } else {
            ROS_INFO("No lead vehicle candidates found.");
            geometry_msgs::Twist status_msg;
            status_msg.linear.x = 0;
            status_msg.linear.y = 0;
            status_msg.linear.z = 0;
            status_msg.angular.z = 0;
            lead_vehicle_status_pub_.publish(status_msg);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lead_vehicle_flag_with_tracking_node");

    CarlaListener listener;
    listener.run();

    return 0;
}
