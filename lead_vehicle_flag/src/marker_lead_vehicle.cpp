// #include <ros/ros.h>
// #include <tf/tf.h>
// #include <math.h>
// #include <vector>
// #include <geometry_msgs/Point.h>
// #include <geometry_msgs/Twist.h>
// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/NavSatFix.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <derived_object_msgs/ObjectArray.h>
// #include <derived_object_msgs/Object.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

// class CarlaListener {
// public:
//     CarlaListener() : nh_("~"), ego_yaw_(0.0) {
//         lead_vehicle_status_pub_ = nh_.advertise<geometry_msgs::Twist>("/carla/lead_vehicle_status", 10);
//         marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/carla/vehicle_markers", 10);

//         // Subscribers
//         gps_sub_ = nh_.subscribe("/carla/hero/GPS", 10, &CarlaListener::gpsCallback, this);
//         imu_sub_ = nh_.subscribe("/carla/hero/IMU", 10, &CarlaListener::imuCallback, this);
//         object_sub_ = nh_.subscribe("/carla/hero/Objects", 10, &CarlaListener::objectCallback, this);
//         position_sub_ = nh_.subscribe("/carla/hero/position", 10, &CarlaListener::positionCallback, this);  // /carla/hero/position 토픽 추가
//     }

//     void run() {
//         ros::spin();
//     }

// private:
//     ros::NodeHandle nh_;
//     ros::Publisher lead_vehicle_status_pub_;
//     ros::Publisher marker_pub_;
//     ros::Subscriber gps_sub_, imu_sub_, object_sub_, position_sub_;

//     double ego_yaw_;
//     geometry_msgs::Point ego_position_;
//     geometry_msgs::Point ego_position_from_topic_;  // /carla/hero/position 토픽에서 받은 위치 저장

//     const double EARTH_RADIUS_EQUA = 6378137.0;  // Earth's radius in meters

//     double deg2rad(double degrees) {
//         return degrees * M_PI / 180.0;
//     }

//     double rad2deg(double radians) {
//         return radians * 180.0 / M_PI;
//     }

//     void llaToENU(double lat, double lon, double alt, double lat0, double lon0, double alt0, double& e, double& n, double& u) {
//         double scale = cos(deg2rad(lat0));

//         double mx = scale * deg2rad(lon) * EARTH_RADIUS_EQUA;
//         double my = scale * EARTH_RADIUS_EQUA * log(tan(M_PI / 4.0 + deg2rad(lat) / 2.0));

//         double mx_ref = scale * deg2rad(lon0) * EARTH_RADIUS_EQUA;
//         double my_ref = scale * EARTH_RADIUS_EQUA * log(tan(M_PI / 4.0 + deg2rad(lat0) / 2.0));

//         e = mx - mx_ref;
//         n = my - my_ref;
//         u = alt - alt0;
//     }

//     double calculateDistance(const geometry_msgs::Point& pos1, const geometry_msgs::Point& pos2) {
//         return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2));
//     }

//     void imuCallback(const sensor_msgs::Imu::ConstPtr& data) {
//         double roll, pitch;
//         tf::Quaternion quaternion(data->orientation.x, data->orientation.y, data->orientation.z, data->orientation.w);
//         tf::Matrix3x3(quaternion).getRPY(roll, pitch, ego_yaw_);
//     }

//     void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& data) {
//         double lat0 = 0.0, lon0 = 0.0, alt0 = 0.0;
//         llaToENU(data->latitude, data->longitude, data->altitude, lat0, lon0, alt0, ego_position_.x, ego_position_.y, ego_position_.z);
//     }

//     void positionCallback(const geometry_msgs::Point::ConstPtr& data) {
//         ego_position_from_topic_ = *data;  // /carla/hero/position 토픽에서 받은 위치를 저장
//         ROS_INFO("Ego_position: x=%.2f, y=%.2f, z=%.2f", ego_position_from_topic_.x, ego_position_from_topic_.y, ego_position_from_topic_.z);
//     }

//     double angleMod(double angle) {
//         return fmod(angle + M_PI, 2 * M_PI) - M_PI;
//     }

//     void objectCallback(const derived_object_msgs::ObjectArray::ConstPtr& data) {
//         std::vector<const derived_object_msgs::Object*> closest_objects(2, nullptr);
//         std::vector<double> min_distances(2, std::numeric_limits<double>::infinity());
//         std::vector<int> candidate_ids;
//         visualization_msgs::MarkerArray marker_array;

//         if (ego_yaw_ != 0) {
//             for (const auto& obj : data->objects) {
//                 // 자차의 위치를 기준으로 상대 위치 계산
//                 double dx = obj.pose.position.x - ego_position_from_topic_.x;
//                 double dy = obj.pose.position.y - ego_position_from_topic_.y;

//                 // 자차의 yaw 값을 고려한 회전 변환 적용
//                 geometry_msgs::Point relative_position;
//                 relative_position.x = dx * cos(ego_yaw_) + dy * sin(ego_yaw_);
//                 relative_position.y = -dx * sin(ego_yaw_) + dy * cos(ego_yaw_);
//                 relative_position.z = obj.pose.position.z - ego_position_from_topic_.z;


//                 // 객체가 자차 전방에 있는지 확인
//                 if (relative_position.x < 0) {
//                     // 객체가 자차의 뒤쪽에 있는 경우
//                     continue;
//                 }

//                 double distance = calculateDistance(ego_position_from_topic_, obj.pose.position);

//                 // 내 차량과 같은 위치인지 확인하여 제외
//                 if (distance < 1.0)
//                     continue;

//                 // classification이 6인지 확인 (Lead Vehicle)
//                 if (obj.classification == 6) {
//                     double obj_yaw = tf::getYaw(obj.pose.orientation);

//                     // 헤딩이 같은 방향으로 향하는 객체 필터링 (예: ±10도 내외)
//                     bool valid_angle = std::abs(angleMod(obj_yaw - ego_yaw_)) < 0.174;

//                     // 모든 객체에 대해 3D Box 생성
//                     visualization_msgs::Marker marker;
//                     marker.header.frame_id = "hero";
//                     marker.header.stamp = ros::Time::now();
//                     marker.ns = "objects";
//                     marker.id = obj.id;
//                     marker.type = visualization_msgs::Marker::CUBE;
//                     marker.action = visualization_msgs::Marker::ADD;

//                     // 자차의 yaw 값을 고려한 상대 위치를 마커의 위치로 설정
//                     marker.pose.position = relative_position;
//                     marker.pose.orientation = obj.pose.orientation;

//                     marker.scale.x = obj.shape.dimensions[0];
//                     marker.scale.y = obj.shape.dimensions[1];
//                     marker.scale.z = obj.shape.dimensions[2];

//                     marker.color.a = 0.5;
//                     marker.color.r = 0.0;
//                     marker.color.g = 0.0;
//                     marker.color.b = 1.0;  // 기본은 파란색

//                     marker_array.markers.push_back(marker);
//                     ROS_INFO("Marker ID: %d, Position: x=%.2f, y=%.2f, z=%.2f, Color: r=%.2f, g=%.2f, b=%.2f, a=%.2f", 
//                                 marker.id, marker.pose.position.x, marker.pose.position.y, marker.pose.position.z, 
//                                 marker.color.r, marker.color.g, marker.color.b, marker.color.a);
//                     if (valid_angle && distance <= 30.0) {  
//                         candidate_ids.push_back(obj.id);
//                         if (distance < min_distances[0]) {
//                             min_distances[1] = min_distances[0];
//                             closest_objects[1] = closest_objects[0];
//                             min_distances[0] = distance;
//                             closest_objects[0] = &obj;
//                         } else if (distance < min_distances[1]) {
//                             min_distances[1] = distance;
//                             closest_objects[1] = &obj;
//                         }
//                     }
//                 }
//             }
//         }

//         // Lead Vehicle을 빨간색으로 표시
//         for (const auto& lead : closest_objects) {
//             if (lead == nullptr) continue;

//             for (auto& marker : marker_array.markers) {
//                 if (marker.id == lead->id) {
//                     marker.color.r = 1.0;
//                     marker.color.g = 0.0;
//                     marker.color.b = 0.0;  // 빨간색으로 변경
//                     marker.color.a = 1.0;  // 불투명으로 변경
//                 }
//             }
//         }

//         marker_pub_.publish(marker_array);

//         if (closest_objects[0] != nullptr) {
//             ROS_INFO("Lead Vehicle 후보군 IDs: ");
//             for (auto id : candidate_ids)
//                 ROS_INFO("ID: %d", id);

//             ROS_INFO("선택된 Lead Vehicle 1 ID: %d", closest_objects[0]->id);
//             if (closest_objects[1] != nullptr) {
//                 ROS_INFO("선택된 Lead Vehicle 2 ID: %d", closest_objects[1]->id);
//             }

//             for (size_t i = 0; i < closest_objects.size(); ++i) {
//                 if (closest_objects[i] == nullptr)
//                     continue;

//                 const auto& obj = *closest_objects[i];

//                 // 위치와 헤딩 및 속도 계산
//                 double position_x = obj.pose.position.x;
//                 double position_y = obj.pose.position.y;

//                 // 속도는 x, y의 선속도를 기반으로 계산
//                 double velocity = std::sqrt(std::pow(obj.twist.linear.x, 2) + std::pow(obj.twist.linear.y, 2));

//                 // 헤딩 계산
//                 double heading_yaw = tf::getYaw(obj.pose.orientation);

//                 // Ego 차량과 Lead 차량 간의 거리 계산
//                 double distance_to_lead = calculateDistance(ego_position_, obj.pose.position);

//                 // Twist 메시지를 사용하여 위치, 속도, 헤딩 퍼블리시
//                 geometry_msgs::Twist status_msg;

//                 // linear.x와 linear.y에 위치 정보를 저장
//                 status_msg.linear.x = position_x;
//                 status_msg.linear.y = position_y;

//                 // linear.z에 속도 정보를 저장
//                 status_msg.linear.z = velocity;

//                 // angular.z에 헤딩 정보를 저장
//                 status_msg.angular.z = heading_yaw;

//                 ROS_INFO("Lead Vehicle %d Position: x=%.2f, y=%.2f", static_cast<int>(i + 1), position_x, position_y);
//                 ROS_INFO("Ego  Vehicle Position: x=%.2f, y=%.2f", ego_position_.x, ego_position_.y);
//                 ROS_INFO("Distance to Lead Vehicle %d: %.2f meters", static_cast<int>(i + 1), distance_to_lead);
//                 ROS_INFO("Lead Vehicle %d Heading: %.2f radians", static_cast<int>(i + 1), heading_yaw);
//                 ROS_INFO("Ego  Vehicle Heading: %.2f radians", ego_yaw_);
//                 ROS_INFO("----------------------------");

//                 lead_vehicle_status_pub_.publish(status_msg);
//             }
//         } else {
//             ROS_INFO("Lead vehicle 후보군이 없습니다.");
//             geometry_msgs::Twist status_msg;

//             // 모든 필드를 0으로 설정
//             status_msg.linear.x = 0;
//             status_msg.linear.y = 0;
//             status_msg.linear.z = 0;
//             status_msg.angular.z = 0;

//             lead_vehicle_status_pub_.publish(status_msg);
//         }
//     }
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "lead_vehicle_flag_node");

//     CarlaListener listener;
//     listener.run();

//     return 0;
// }

#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <derived_object_msgs/ObjectArray.h>
#include <derived_object_msgs/Object.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>

class CarlaListener {
public:
    CarlaListener() : nh_("~"), ego_yaw_(0.0), ego_pitch_(0.0), ego_roll_(0.0) {
        lead_vehicle_status_pub_ = nh_.advertise<geometry_msgs::Twist>("/carla/lead_vehicle_status", 10);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/carla/vehicle_markers", 10);

        // Subscribers
        gps_sub_ = nh_.subscribe("/carla/hero/GPS", 10, &CarlaListener::gpsCallback, this);
        imu_sub_ = nh_.subscribe("/carla/hero/IMU", 10, &CarlaListener::imuCallback, this);
        object_sub_ = nh_.subscribe("/carla/hero/Objects", 10, &CarlaListener::objectCallback, this);
        position_sub_ = nh_.subscribe("/carla/hero/position", 10, &CarlaListener::positionCallback, this);  // /carla/hero/position 토픽 추가
    }

    void run() {
        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher lead_vehicle_status_pub_;
    ros::Publisher marker_pub_;
    ros::Subscriber gps_sub_, imu_sub_, object_sub_, position_sub_;

    double ego_yaw_, ego_pitch_, ego_roll_;
    geometry_msgs::Point ego_position_;
    geometry_msgs::Point ego_position_from_topic_;  // /carla/hero/position 토픽에서 받은 위치 저장

    const double EARTH_RADIUS_EQUA = 6378137.0;  // Earth's radius in meters

    double deg2rad(double degrees) {
        return degrees * M_PI / 180.0;
    }

    double rad2deg(double radians) {
        return radians * 180.0 / M_PI;
    }

    void llaToENU(double lat, double lon, double alt, double lat0, double lon0, double alt0, double& e, double& n, double& u) {
        double scale = cos(deg2rad(lat0));

        double mx = scale * deg2rad(lon) * EARTH_RADIUS_EQUA;
        double my = scale * EARTH_RADIUS_EQUA * log(tan(M_PI / 4.0 + deg2rad(lat) / 2.0));

        double mx_ref = scale * deg2rad(lon0) * EARTH_RADIUS_EQUA;
        double my_ref = scale * EARTH_RADIUS_EQUA * log(tan(M_PI / 4.0 + deg2rad(lat0) / 2.0));

        e = mx - mx_ref;
        n = my - my_ref;
        u = alt - alt0;
    }

    double calculateDistance(const geometry_msgs::Point& pos1, const geometry_msgs::Point& pos2) {
        return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2));
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& data) {
        tf::Quaternion quaternion(data->orientation.x, data->orientation.y, data->orientation.z, data->orientation.w);
        tf::Matrix3x3(quaternion).getRPY(ego_roll_, ego_pitch_, ego_yaw_);

    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& data) {
        double lat0 = 0.0, lon0 = 0.0, alt0 = 0.0;
        llaToENU(data->latitude, data->longitude, data->altitude, lat0, lon0, alt0, ego_position_.x, ego_position_.y, ego_position_.z);
    }

    void positionCallback(const geometry_msgs::Point::ConstPtr& data) {
        ego_position_from_topic_ = *data;  // /carla/hero/position 토픽에서 받은 위치를 저장
        ROS_INFO("Ego_position: x=%.2f, y=%.2f, z=%.2f", ego_position_from_topic_.x, ego_position_from_topic_.y, ego_position_from_topic_.z);
    }

    double angleMod(double angle) {
        return fmod(angle + M_PI, 2 * M_PI) - M_PI;
    }

    void objectCallback(const derived_object_msgs::ObjectArray::ConstPtr& data) {
        std::vector<const derived_object_msgs::Object*> closest_objects(2, nullptr);
        std::vector<double> min_distances(2, std::numeric_limits<double>::infinity());
        std::vector<int> candidate_ids;
        visualization_msgs::MarkerArray marker_array;

        if (ego_yaw_ != 0) {
            // Rotation matrix based on roll, pitch, yaw
            Eigen::Matrix3d rotation_matrix;
            rotation_matrix = Eigen::AngleAxisd(ego_yaw_, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(ego_pitch_, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(ego_roll_, Eigen::Vector3d::UnitX());

            // Inverse rotation matrix
            Eigen::Matrix3d inv_rotation_matrix = rotation_matrix.inverse();

            for (const auto& obj : data->objects) {
                // 자차의 위치를 기준으로 상대 위치 계산
                Eigen::Vector3d relative_position_vector(obj.pose.position.x - ego_position_from_topic_.x,
                                                         obj.pose.position.y - ego_position_from_topic_.y,
                                                         obj.pose.position.z - ego_position_from_topic_.z);

                // Apply inverse rotation matrix to position
                Eigen::Vector3d transformed_position = inv_rotation_matrix * relative_position_vector;

                geometry_msgs::Point relative_position;
                relative_position.x = transformed_position.x();
                relative_position.y = transformed_position.y();
                relative_position.z = transformed_position.z();

                // 객체가 자차 전방에 있는지 확인
                if (relative_position.x < 0) {
                    // 객체가 자차의 뒤쪽에 있는 경우
                    continue;
                }

                double distance = calculateDistance(ego_position_from_topic_, obj.pose.position);

                // 내 차량과 같은 위치인지 확인하여 제외
                if (distance < 1.0)
                    continue;
                
                
                if (std::abs(relative_position.y) > 1.25){
                    continue;
                }

                // classification이 6인지 확인 (Lead Vehicle)
                if (obj.classification == 6) {
                    double obj_yaw = tf::getYaw(obj.pose.orientation);

                    // 헤딩이 같은 방향으로 향하는 객체 필터링 (예: ±10도 내외)
                    bool valid_angle = std::abs(angleMod(obj_yaw - ego_yaw_)) < 0.174;

                    // Scale transformation
                    Eigen::Vector3d original_scale(obj.shape.dimensions[0],
                                                   obj.shape.dimensions[1],
                                                   obj.shape.dimensions[2]);

                    Eigen::Vector3d transformed_scale = inv_rotation_matrix * original_scale;

                    // 모든 객체에 대해 3D Box 생성
                    visualization_msgs::Marker marker;
                    marker.header.frame_id = "hero";
                    marker.header.stamp = ros::Time::now();
                    marker.ns = "objects";
                    marker.id = obj.id;
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.action = visualization_msgs::Marker::ADD;

                    // 변환된 상대 위치를 마커의 위치로 설정
                    marker.pose.position = relative_position;
                    marker.pose.orientation = obj.pose.orientation;

                    // 변환된 스케일을 마커의 스케일로 설정
                    marker.scale.x = std::abs(transformed_scale.x());
                    marker.scale.y = std::abs(transformed_scale.y());
                    marker.scale.z = std::abs(transformed_scale.z());

                    marker.color.a = 0.5;
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;  // 기본은 파란색
                    marker.lifetime = ros::Duration(1.0);
                    marker_array.markers.push_back(marker);
                    ROS_INFO("Marker ID: %d, Position: x=%.2f, y=%.2f, z=%.2f, Scale: x=%.2f, y=%.2f, z=%.2f, Color: r=%.2f, g=%.2f, b=%.2f, a=%.2f", 
                                marker.id, marker.pose.position.x, marker.pose.position.y, marker.pose.position.z,
                                marker.scale.x, marker.scale.y, marker.scale.z,
                                marker.color.r, marker.color.g, marker.color.b, marker.color.a);
                    if (valid_angle && distance <= 30.0) {  
                        candidate_ids.push_back(obj.id);
                        if (distance < min_distances[0]) {
                            min_distances[1] = min_distances[0];
                            closest_objects[1] = closest_objects[0];
                            min_distances[0] = distance;
                            closest_objects[0] = &obj;
                        } else if (distance < min_distances[1]) {
                            min_distances[1] = distance;
                            closest_objects[1] = &obj;
                        }
                    }
                }
            }
        }

        // Lead Vehicle을 빨간색으로 표시
        for (const auto& lead : closest_objects) {
            if (lead == nullptr) continue;

            for (auto& marker : marker_array.markers) {
                if (marker.id == lead->id) {
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;  // 빨간색으로 변경
                    marker.color.a = 1.0;  // 불투명으로 변경
                }
            }
        }
        
        marker_pub_.publish(marker_array);

        if (closest_objects[0] != nullptr) {
            ROS_INFO("Lead Vehicle 후보군 IDs: ");
            for (auto id : candidate_ids)
                ROS_INFO("ID: %d", id);

            ROS_INFO("선택된 Lead Vehicle 1 ID: %d", closest_objects[0]->id);
            if (closest_objects[1] != nullptr) {
                ROS_INFO("선택된 Lead Vehicle 2 ID: %d", closest_objects[1]->id);
            }

            for (size_t i = 0; i < closest_objects.size(); ++i) {
                if (closest_objects[i] == nullptr)
                    continue;

                const auto& obj = *closest_objects[i];

                // 위치와 헤딩 및 속도 계산
                double position_x = obj.pose.position.x;
                double position_y = obj.pose.position.y;

                // 속도는 x, y의 선속도를 기반으로 계산
                double velocity = std::sqrt(std::pow(obj.twist.linear.x, 2) + std::pow(obj.twist.linear.y, 2));

                // 헤딩 계산
                double heading_yaw = tf::getYaw(obj.pose.orientation);

                // Ego 차량과 Lead 차량 간의 거리 계산
                double distance_to_lead = calculateDistance(ego_position_, obj.pose.position);

                // Twist 메시지를 사용하여 위치, 속도, 헤딩 퍼블리시
                geometry_msgs::Twist status_msg;

                // linear.x와 linear.y에 위치 정보를 저장
                status_msg.linear.x = position_x;
                status_msg.linear.y = position_y;

                // linear.z에 속도 정보를 저장
                status_msg.linear.z = velocity;

                // angular.z에 헤딩 정보를 저장
                status_msg.angular.z = heading_yaw;

                ROS_INFO("Lead Vehicle %d Position: x=%.2f, y=%.2f", static_cast<int>(i + 1), position_x, position_y);
                ROS_INFO("Ego  Vehicle Position: x=%.2f, y=%.2f", ego_position_.x, ego_position_.y);
                ROS_INFO("Distance to Lead Vehicle %d: %.2f meters", static_cast<int>(i + 1), distance_to_lead);
                ROS_INFO("Lead Vehicle %d Heading: %.2f radians", static_cast<int>(i + 1), heading_yaw);
                ROS_INFO("Ego  Vehicle Heading: %.2f radians", ego_yaw_);
                ROS_INFO("----------------------------");

                lead_vehicle_status_pub_.publish(status_msg);
            }
        } else {
            ROS_INFO("Lead vehicle 후보군이 없습니다.");
            geometry_msgs::Twist status_msg;

            // 모든 필드를 0으로 설정
            status_msg.linear.x = 0;
            status_msg.linear.y = 0;
            status_msg.linear.z = 0;
            status_msg.angular.z = 0;

            lead_vehicle_status_pub_.publish(status_msg);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lead_vehicle_flag_node");

    CarlaListener listener;
    listener.run();

    return 0;
}
