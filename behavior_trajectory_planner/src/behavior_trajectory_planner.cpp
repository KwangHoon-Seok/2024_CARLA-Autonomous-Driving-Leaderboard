#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <carla_msgs/CarlaSpeedometer.h>
#include <sensor_msgs/Imu.h>
#include <cmath>
#include <behavior_trajectory_planner/TrajectoryPoint.h>
#include <behavior_trajectory_planner/TrajectoryArray.h>
#include <behavior_trajectory_planner/DetectionInfo.h>
#include <behavior_trajectory_planner/DetectionInfoArray.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <sstream> 
#include <vector>
#include <string>
#include <chrono>
#include <tf/tf.h> // For quaternion to RPY conversion
#include <Eigen/Dense> // For Eigen types

using namespace std;

// Define constants
// Path Command
const int LANE_FOLLOWING = 100;
const int LANE_CHANGE = 101;
const int AVOIDANCE = 102;
// Road Option
const int SHOULDER = 0; // 20km/h
const int TWO_LANE = 1; // 40km/h
const int STRAIGHT = 2; // 25km/h
const int TURN = 3; // 15km/h

// IDs for detection
const int change_sign_class_id = 0;  
const int door_class_id = 1; 

class BehaviorTrajectoryPlanner {
public:
    BehaviorTrajectoryPlanner() {
        ros::NodeHandle nh;
        
        class_sub = nh.subscribe("/detection_with_distance", 1000, &BehaviorTrajectoryPlanner::class_callback, this);
        lane_change_done_sub = nh.subscribe("/lane_change_done", 1000, &BehaviorTrajectoryPlanner::lane_change_done_callback, this);
        local_planning_sub = nh.subscribe("/local_planning", 1000, &BehaviorTrajectoryPlanner::local_planning_callback, this);
        current_point_index_sub = nh.subscribe("/current_point_index", 1000, &BehaviorTrajectoryPlanner::current_point_index_callback, this);
        enu_sub = nh.subscribe("/enu_coordinates", 1000, &BehaviorTrajectoryPlanner::enu_callback, this);
        speed_sub = nh.subscribe("/carla/hero/Speed", 1000, &BehaviorTrajectoryPlanner::speed_callback, this);
        yaw_rate_sub = nh.subscribe("carla/hero/IMU", 1000, &BehaviorTrajectoryPlanner::yaw_rate_callback, this);
        gt_position_sub = nh.subscribe("/carla/hero/position", 1000, &BehaviorTrajectoryPlanner::position_callback, this);

        trajectory_pub = nh.advertise<std_msgs::String>("/trajectory_path", 10);
        ego_trajectory_pub = nh.advertise<behavior_trajectory_planner::TrajectoryArray>("/ego_trajectory", 10);
        current_road_option_pub = nh.advertise<std_msgs::Int32>("/current_road_option", 10);
        local_planning_pub = nh.advertise<std_msgs::Bool>("/local_planning", 10);
        marker_pub = nh.advertise<visualization_msgs::Marker>("/ego_trajectory_visualization", 1000);

        path_command = LANE_FOLLOWING;
        current_trajectory_path = "/home/ailab/catkin_ws/src/carla_ros/path/global_path.csv";
        
        alternative_paths = {
            "/catkin_ws/src/carla_ros/path/accident1.csv",
            "/catkin_ws/src/carla_ros/path/accident2.csv",
            "/catkin_ws/src/carla_ros/path/accident3.csv",
            "/catkin_ws/src/carla_ros/path/accident4.csv",
            "/catkin_ws/src/carla_ros/path/construction.csv",
        };
        
        path_index = 0;
        current_point_index = 0;
        in_lane_change = false;
        
        lane_change_done = false;
        local_planning = false;
        
        x = 0.0;
        y = 0.0;
        yaw = 0.0;
        yaw_rate = 0.0;
        speed = 0.0;
        
        in_avoidance_mode = false;  // Avoidance 상태 플래그 초기화
    }

    // Callback functions
    void class_callback(const behavior_trajectory_planner::DetectionInfoArray::ConstPtr& msg) {
        path_command = LANE_FOLLOWING;
        // Iterate through each detection in the array
        for (const auto& detection : msg->detections) {
            if (detection.class_id == change_sign_class_id) {
                if (detection.distance < 10.0) {
                    path_command = LANE_CHANGE;
                    ROS_INFO("Change sign detected.");
                }
            } 
            else if (detection.class_id == door_class_id) {
                if (detection.distance < 30.0) {
                    path_command = AVOIDANCE;
                    ROS_INFO("Door detected.");
                }
            }
        }
    }

    void lane_change_done_callback(const std_msgs::Bool::ConstPtr& msg) {
        lane_change_done = msg->data;
    }

    void local_planning_callback(const std_msgs::Bool::ConstPtr& msg) {
        local_planning = msg->data;
    }

    void current_point_index_callback(const std_msgs::Int32::ConstPtr& msg) {
        current_point_index = msg->data;
    }

    void enu_callback(const geometry_msgs::Point::ConstPtr& msg) {
        x = msg->x;
        y = msg->y;
        yaw = msg->z;
    }

    void position_callback(const geometry_msgs::Point::ConstPtr& msg) {
        x = msg->x;
        y = msg->y;
    }

    void yaw_rate_callback(const sensor_msgs::Imu::ConstPtr& msg) {
        yaw_rate = msg->angular_velocity.z; // rad/s

        tf::Quaternion quat(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        this->yaw = tf::getYaw(quat); // radian
        

        while (this->yaw > M_PI) this->yaw -= 2.0 * M_PI;
        while (this->yaw < -M_PI) this->yaw += 2.0 * M_PI;
    }

    void speed_callback(const carla_msgs::CarlaSpeedometer::ConstPtr& msg) {
        speed = msg->speed;
    }

    vector<Eigen::Vector2d> calculate_circles(const Eigen::Vector2d& position, double roll, double pitch, double yaw, double vehicle_width, double vehicle_length, double radius) {
        vector<Eigen::Vector2d> circles; // [x,y] vector containing 3 points

        // Vehicle's center position (ENU)
        Eigen::Vector2d center(position.x(), position.y());

        // Define the transformation matrix using roll, pitch, yaw (RPY)
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

        // Inverse of the transformation matrix
        Eigen::Matrix3d R_inv = R.inverse();

        // Define points in the vehicle's local frame (assuming z=0 for a 2D transformation)
        Eigen::Vector3d front_point(2.3, 0.0, 0.0); // x: +2.3, y: 0
        Eigen::Vector3d center_point(0.0, 0.0, 0.0); // x: 0, y: 0
        Eigen::Vector3d rear_point(-2.3, 0.0, 0.0); // x: -2.3, y: 0

        // Transform the points to the global frame using the inverse transformation
        Eigen::Vector3d front_global = R_inv * front_point;
        Eigen::Vector3d center_global = R_inv * center_point;
        Eigen::Vector3d rear_global = R_inv * rear_point;

        // Convert the 3D points to 2D by ignoring the z component
        Eigen::Vector2d front_circle = center + Eigen::Vector2d(front_global.x(), front_global.y());
        Eigen::Vector2d center_circle = center + Eigen::Vector2d(center_global.x(), center_global.y());
        Eigen::Vector2d rear_circle = center + Eigen::Vector2d(rear_global.x(), rear_global.y());

        // Store the circle positions in the vector
        circles.push_back(front_circle);
        circles.push_back(center_circle);
        circles.push_back(rear_circle);

        return circles;
    }

    void visualize_trajectory(const behavior_trajectory_planner::TrajectoryArray& trajectory) {
        // Line strip for the trajectory (thicker line)
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "hero";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "trajectory_lines";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;

        line_strip.id = 1;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = 0.15; // Thickness of the line

        // Line color (light green with higher opacity)
        line_strip.color.r = 0.0;
        line_strip.color.g = 1.0;
        line_strip.color.b = 0.0;
        line_strip.color.a = 0.8; // Higher opacity

        // Populate the points in the line
        for (const auto& point : trajectory.points) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0; // On the ground (z=0)
            line_strip.points.push_back(p);
        }

        // Publish the thicker line strip
        marker_pub.publish(line_strip);

        // Additional line strip for the trajectory (highlighter effect)
        visualization_msgs::Marker highlight_strip;
        highlight_strip.header.frame_id = "hero";
        highlight_strip.header.stamp = ros::Time::now();
        highlight_strip.ns = "trajectory_highlighter";
        highlight_strip.action = visualization_msgs::Marker::ADD;
        highlight_strip.pose.orientation.w = 1.0;

        highlight_strip.id = 2;
        highlight_strip.type = visualization_msgs::Marker::LINE_STRIP;
        highlight_strip.scale.x = 1.8; // Thickness of the highlight line (wider)

        // Line color (light green with lower opacity for the highlighter effect)
        highlight_strip.color.r = 0.0;
        highlight_strip.color.g = 1.0;
        highlight_strip.color.b = 0.0;
        highlight_strip.color.a = 0.05; // Lower opacity for highlighter effect

        // Populate the points in the highlight line with z fixed to 0
        for (const auto& point : trajectory.points) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0; // Force z to be 0, fixing the line to the ground
            highlight_strip.points.push_back(p);
        }

        // Publish the highlight line strip
        marker_pub.publish(highlight_strip);

        // Circles (spheres) at each point with filled color and outline
        int point_id = 1000; // Start ID for the points
        for (const auto& point : trajectory.points) {
            // Sphere for the fill
            visualization_msgs::Marker circle_fill;
            circle_fill.header.frame_id = "hero";
            circle_fill.header.stamp = ros::Time::now();
            circle_fill.ns = "trajectory_points_fill";
            circle_fill.action = visualization_msgs::Marker::ADD;
            circle_fill.pose.orientation.w = 1.0;

            circle_fill.id = point_id++;
            circle_fill.type = visualization_msgs::Marker::SPHERE; // Use SPHERE for the filled circle
            circle_fill.scale.x = 2.0; // Diameter of the circle
            circle_fill.scale.y = 2.0;
            circle_fill.scale.z = 0.01; // Very thin, almost flat on the ground

            // Fill color (green with lower opacity)
            circle_fill.color.r = 0.0;
            circle_fill.color.g = 1.0;
            circle_fill.color.b = 0.0;
            circle_fill.color.a = 0.3; // Semi-transparent

            // Set the position of the circle
            circle_fill.pose.position.x = point.x;
            circle_fill.pose.position.y = point.y;
            circle_fill.pose.position.z = 0; // On the ground (z=0)

            // Publish the filled circle
            marker_pub.publish(circle_fill);

            // Outline for the circle
            visualization_msgs::Marker circle_outline;
            circle_outline.header.frame_id = "hero";
            circle_outline.header.stamp = ros::Time::now();
            circle_outline.ns = "trajectory_points_outline";
            circle_outline.action = visualization_msgs::Marker::ADD;
            circle_outline.pose.orientation.w = 1.0;

            circle_outline.id = point_id++;
            circle_outline.type = visualization_msgs::Marker::LINE_STRIP; // Use LINE_STRIP for the circle outline
            circle_outline.scale.x = 0.02; // Thickness of the outline
            circle_outline.lifetime = ros::Duration(1.0);

            // Outline color (dark green with full opacity)
            circle_outline.color.r = 0.0;
            circle_outline.color.g = 0.5; // Dark green
            circle_outline.color.b = 0.0;
            circle_outline.color.a = 1.0; // Full opacity

            // Generate points for the circle outline
            int num_points = 36; // Number of points in the circle
            double radius = 1.0;
            for (int i = 0; i <= num_points; ++i) {
                double angle = 2 * M_PI * i / num_points;
                geometry_msgs::Point p;
                p.x = point.x + radius * cos(angle);
                p.y = point.y + radius * sin(angle);
                p.z = 0; // On the ground (z=0)
                circle_outline.points.push_back(p);
            }

            // Publish the circle outline
            marker_pub.publish(circle_outline);
        }
    }

    // Function to predict trajectory in vehicle coordinates and then convert to ENU coordinates
    behavior_trajectory_planner::TrajectoryArray predict_trajectory_ctrv(double x0, double y0, double theta0, double v0, double omega0, double T, double dt) {
        // For leaving the parking area - 얼추 인덱스 물어보기
        if (current_point_index > 7 && current_point_index < 40) {
            v0 = 2.3; // Regard as 10km/h
        }

        // 필요하다면 lane change 상황에서도 위와 같은 작업이 필요할 수 있음
        
        // Set Constraint velocity
        if (v0 < 1.5) {
            v0 = 1.5;
        }

        behavior_trajectory_planner::TrajectoryArray trajectory;

        double max_yaw_rate = M_PI / 1.8 / T; // Maximum yaw rate that corresponds to pi/3.5 rad in 3 seconds
        omega0 = std::clamp(omega0, -max_yaw_rate, max_yaw_rate); // Clamp omega0 to be within the range [-max_yaw_rate, max_yaw_rate]

        double t = 0.0;

        while (t <= T) {
            behavior_trajectory_planner::TrajectoryPoint point;

            double delta_theta = omega0 * t; // Yaw change in time t
            double theta_t = theta0 + delta_theta; // Absolute yaw in ENU coordinates

            double delta_x, delta_y;

            if (fabs(omega0) > 0.2) { // If the yaw rate is significant, use a circular motion model
                delta_x = (v0 / omega0) * (sin(theta_t) - sin(theta0));
                delta_y = (v0 / omega0) * (-cos(theta_t) + cos(theta0));
            } else { // If the yaw rate is small, approximate with a straight line
                delta_x = v0 * t * cos(theta0);
                delta_y = v0 * t * sin(theta0);
            }

            point.x = x0 + delta_x; // Convert to global ENU coordinates
            point.y = y0 + delta_y; // Convert to global ENU coordinates
            point.yaw = theta_t; // Yaw in global ENU coordinates
            point.time = t;

            trajectory.points.push_back(point);
            t += dt;
        }

        return trajectory;
    }

    int get_road_option_from_csv(const std::string& path, int index) {
        std::ifstream file(path);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open file: %s", path.c_str());
            return SHOULDER;
        }

        std::string line;
        int current_index = SHOULDER;

        while (std::getline(file, line)) {
            if (current_index == index) {
                std::stringstream ss(line);
                std::string token;
                int column_count = 0;

                while (std::getline(ss, token, ',')) {
                    if (column_count == 2) {
                        file.close();
                        return std::stoi(token);
                    }
                    column_count++;
                }
            }
            current_index++;
        }

        file.close();
        ROS_ERROR("Index %d out of range in file: %s", index, path.c_str());
        return SHOULDER;
    }

    void run() {
        ros::Rate rate(20);

        while (ros::ok()) {
            auto start_time = std::chrono::high_resolution_clock::now();

            std::string previous_trajectory = current_trajectory_path;

            // Lane change logic
            if (lane_change_done) {
                in_lane_change = false;
                path_index++;
            }

            // Check if we should enter or stay in avoidance mode
            if (path_command == AVOIDANCE || (local_planning && !in_avoidance_mode)) {
                ROS_INFO("Entering Avoidance Mode");
                in_avoidance_mode = true;
                local_planning = true;
                current_trajectory_path = "/home/ailab/catkin_ws/src/carla_ros/path/global_path.csv";
            } 
            // If in avoidance mode, continue avoidance
            else if (in_avoidance_mode) {
                ROS_INFO("Path Command : Avoidance - Continuing");
                current_trajectory_path = "/home/ailab/catkin_ws/src/carla_ros/path/global_path.csv";
                
                // Check if we should exit avoidance mode
                if (local_planning == false) {
                    ROS_INFO("Exiting Avoidance Mode");
                    in_avoidance_mode = false;
                    local_planning = false;
                }
            }
            // Regular lane following or other commands
            else {
                ROS_INFO("Path Command : Lane Following");
                current_trajectory_path = "/home/ailab/catkin_ws/src/carla_ros/path/global_path.csv";
            }

            // Publish the trajectory path
            std_msgs::String trajectory_msg;
            trajectory_msg.data = current_trajectory_path;
            trajectory_pub.publish(trajectory_msg);

            // Publish the current road option if on the main path
            int current_road_option = SHOULDER;
            if (current_trajectory_path == "/home/ailab/catkin_ws/src/carla_ros/path/global_path.csv") {
                current_road_option = get_road_option_from_csv(current_trajectory_path, current_point_index);
            }
            std_msgs::Int32 road_option_msg;
            road_option_msg.data = current_road_option;
            current_road_option_pub.publish(road_option_msg);

            std_msgs::Bool local_planning_msg;
            local_planning_msg.data = local_planning;
            local_planning_pub.publish(local_planning_msg);

            // Predict the ego trajectory and visualize
            float time_horizon = (current_road_option == TURN) ? 2.0 : 3.0;
            behavior_trajectory_planner::TrajectoryArray ego_trajectory = predict_trajectory_ctrv(x, y, yaw, speed, yaw_rate, time_horizon, 0.2);
            visualize_trajectory(ego_trajectory);
            ego_trajectory_pub.publish(ego_trajectory);

            // Log and loop timing
            if (previous_trajectory != current_trajectory_path) {
                ROS_INFO("Trajectory Path updated to: %s", current_trajectory_path.c_str());
            }

            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end_time - start_time;
            ROS_INFO("Loop execution time: %f seconds", elapsed.count());

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::Subscriber class_sub;
    ros::Subscriber lane_change_done_sub;
    ros::Subscriber local_planning_sub;
    ros::Subscriber current_point_index_sub;
    ros::Subscriber enu_sub;
    ros::Subscriber speed_sub;
    ros::Subscriber yaw_rate_sub;
    ros::Subscriber gt_position_sub;
    
    ros::Publisher trajectory_pub;
    ros::Publisher ego_trajectory_pub;
    ros::Publisher current_road_option_pub;
    ros::Publisher local_planning_pub;
    ros::Publisher marker_pub;

    int path_command;
    std::string current_trajectory_path;
    std::vector<std::string> alternative_paths;
    int path_index;
    int current_point_index;
    bool in_lane_change;
    bool lane_change_done;
    bool local_planning;
    
    // Avoidance mode 플래그
    bool in_avoidance_mode;

    double x, y, yaw, pitch, roll, yaw_rate, speed;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "behavior_trajectory_planner");
    BehaviorTrajectoryPlanner planner;
    planner.run();
    return 0;
}


// #include <ros/ros.h>
// #include <std_msgs/Int32.h>
// #include <std_msgs/String.h>
// #include <std_msgs/Bool.h>
// #include <geometry_msgs/Point.h>
// #include <carla_msgs/CarlaSpeedometer.h>
// #include <sensor_msgs/Imu.h>
// #include <cmath>
// #include <behavior_trajectory_planner/TrajectoryPoint.h>
// #include <behavior_trajectory_planner/TrajectoryArray.h>
// #include <behavior_trajectory_planner/DetectionInfo.h>
// #include <behavior_trajectory_planner/DetectionInfoArray.h>
// #include <visualization_msgs/Marker.h>
// #include <fstream>
// #include <sstream> 
// #include <vector>
// #include <string>
// #include <chrono>
// #include <tf/tf.h> // For quaternion to RPY conversion
// #include <Eigen/Dense> // For Eigen types

// using namespace std;

// // Define constants
// // Path Command
// const int LANE_FOLLOWING = 100;
// const int LANE_CHANGE = 101;
// const int AVOIDANCE = 102;
// // Road Option
// const int SHOULDER = 0; // 20km/h
// const int TWO_LANE = 1; // 40km/h
// const int STRAIGHT = 2; // 25km/h
// const int TURN = 3; // 15km/h

// // IDs for detection
// const int change_sign_class_id = 0;  
// const int door_class_id = 1; 

// class BehaviorTrajectoryPlanner {
// public:
//     BehaviorTrajectoryPlanner() {
//         ros::NodeHandle nh;
        
//         class_sub = nh.subscribe("/detection_with_distance", 1000, &BehaviorTrajectoryPlanner::class_callback, this);
//         lane_change_done_sub = nh.subscribe("/lane_change_done", 1000, &BehaviorTrajectoryPlanner::lane_change_done_callback, this);
//         local_planning_sub = nh.subscribe("/local_planning", 1000, &BehaviorTrajectoryPlanner::local_planning_callback, this);
//         current_point_index_sub = nh.subscribe("/current_point_index", 1000, &BehaviorTrajectoryPlanner::current_point_index_callback, this);
//         enu_sub = nh.subscribe("/enu_coordinates", 1000, &BehaviorTrajectoryPlanner::enu_callback, this);
//         speed_sub = nh.subscribe("/carla/hero/Speed", 1000, &BehaviorTrajectoryPlanner::speed_callback, this);
//         yaw_rate_sub = nh.subscribe("carla/hero/IMU", 1000, &BehaviorTrajectoryPlanner::yaw_rate_callback, this);
//         gt_position_sub = nh.subscribe("/carla/hero/position", 1000, &BehaviorTrajectoryPlanner::position_callback, this);

//         trajectory_pub = nh.advertise<std_msgs::String>("/trajectory_path", 10);
//         ego_trajectory_pub = nh.advertise<behavior_trajectory_planner::TrajectoryArray>("/ego_trajectory", 10);
//         current_road_option_pub = nh.advertise<std_msgs::Int32>("/current_road_option", 10);
//         local_planning_pub = nh.advertise<std_msgs::Bool>("/local_planning", 10);
//         marker_pub = nh.advertise<visualization_msgs::Marker>("/ego_trajectory_visualization", 1000);

//         path_command = LANE_FOLLOWING;
//         current_trajectory_path = "/home/ailab/catkin_ws/src/carla_ros/path/global_path.csv";
        
//         alternative_paths = {
//             "/catkin_ws/src/carla_ros/path/accident1.csv",
//             "/catkin_ws/src/carla_ros/path/accident2.csv",
//             "/catkin_ws/src/carla_ros/path/accident3.csv"
//             "/catkin_ws/src/carla_ros/path/accident4.csv",
//             "/catkin_ws/src/carla_ros/path/construction.csv",
//         };
        
//         path_index = 0;
//         current_point_index = 0;
//         in_lane_change = false;
        
//         lane_change_done = false;
//         local_planning = false;
        
//         x = 0.0;
//         y = 0.0;
//         yaw = 0.0;
//         yaw_rate = 0.0;
//         speed = 0.0;
//     }

//     // Callback functions
//     void class_callback(const behavior_trajectory_planner::DetectionInfoArray::ConstPtr& msg) {
//         path_command = LANE_FOLLOWING;
//         // Iterate through each detection in the array
//         for (const auto& detection : msg->detections) {
//             if (detection.class_id == change_sign_class_id) {
//                 if (detection.distance < 10.0) {
//                     path_command = LANE_CHANGE;
//                     ROS_INFO("Change sign detected.");
//                 }
//             } 
//             else if (detection.class_id == door_class_id) {
//                 if (detection.distance < 30.0) {
//                     path_command = AVOIDANCE;
//                     ROS_INFO("Door detected.");
//                 }
//             }
//         }
//     }

//     void lane_change_done_callback(const std_msgs::Bool::ConstPtr& msg) {
//         lane_change_done = msg->data;
//     }

//     void local_planning_callback(const std_msgs::Bool::ConstPtr& msg) {
//         local_planning = msg->data;
//     }

//     void current_point_index_callback(const std_msgs::Int32::ConstPtr& msg) {
//         current_point_index = msg->data;
//     }

//     void enu_callback(const geometry_msgs::Point::ConstPtr& msg) {
//         x = msg->x;
//         y = msg->y;
//         yaw = msg->z;
//     }

//     void position_callback(const geometry_msgs::Point::ConstPtr& msg) {
//         x = msg->x;
//         y = msg->y;
//     }

//     void yaw_rate_callback(const sensor_msgs::Imu::ConstPtr& msg) {
//         yaw_rate = msg->angular_velocity.z; // rad/s

//         tf::Quaternion quat(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
//         this->yaw = tf::getYaw(quat); // radian
        

//         while (this->yaw > M_PI) this->yaw -= 2.0 * M_PI;
//         while (this->yaw < -M_PI) this->yaw += 2.0 * M_PI;
//     }

//     void speed_callback(const carla_msgs::CarlaSpeedometer::ConstPtr& msg) {
//         speed = msg->speed;
//     }

//     vector<Eigen::Vector2d> calculate_circles(const Eigen::Vector2d& position, double roll, double pitch, double yaw, double vehicle_width, double vehicle_length, double radius) {
//         vector<Eigen::Vector2d> circles; // [x,y] vector containing 3 points

//         // Vehicle's center position (ENU)
//         Eigen::Vector2d center(position.x(), position.y());

//         // Define the transformation matrix using roll, pitch, yaw (RPY)
//         Eigen::Matrix3d R;
//         R = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()) *
//             Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY()) *
//             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

//         // Inverse of the transformation matrix
//         Eigen::Matrix3d R_inv = R.inverse();

//         // Define points in the vehicle's local frame (assuming z=0 for a 2D transformation)
//         Eigen::Vector3d front_point(2.3, 0.0, 0.0); // x: +2.3, y: 0
//         Eigen::Vector3d center_point(0.0, 0.0, 0.0); // x: 0, y: 0
//         Eigen::Vector3d rear_point(-2.3, 0.0, 0.0); // x: -2.3, y: 0

//         // Transform the points to the global frame using the inverse transformation
//         Eigen::Vector3d front_global = R_inv * front_point;
//         Eigen::Vector3d center_global = R_inv * center_point;
//         Eigen::Vector3d rear_global = R_inv * rear_point;

//         // Convert the 3D points to 2D by ignoring the z component
//         Eigen::Vector2d front_circle = center + Eigen::Vector2d(front_global.x(), front_global.y());
//         Eigen::Vector2d center_circle = center + Eigen::Vector2d(center_global.x(), center_global.y());
//         Eigen::Vector2d rear_circle = center + Eigen::Vector2d(rear_global.x(), rear_global.y());

//         // Store the circle positions in the vector
//         circles.push_back(front_circle);
//         circles.push_back(center_circle);
//         circles.push_back(rear_circle);

//         return circles;
//     }

//     void visualize_trajectory(const behavior_trajectory_planner::TrajectoryArray& trajectory) {
//         // Line strip for the trajectory (thicker line)
//         visualization_msgs::Marker line_strip;
//         line_strip.header.frame_id = "hero";
//         line_strip.header.stamp = ros::Time::now();
//         line_strip.ns = "trajectory_lines";
//         line_strip.action = visualization_msgs::Marker::ADD;
//         line_strip.pose.orientation.w = 1.0;

//         line_strip.id = 1;
//         line_strip.type = visualization_msgs::Marker::LINE_STRIP;
//         line_strip.scale.x = 0.15; // Thickness of the line

//         // Line color (light green with higher opacity)
//         line_strip.color.r = 0.0;
//         line_strip.color.g = 1.0;
//         line_strip.color.b = 0.0;
//         line_strip.color.a = 0.8; // Higher opacity

//         // Populate the points in the line
//         for (const auto& point : trajectory.points) {
//             geometry_msgs::Point p;
//             p.x = point.x;
//             p.y = point.y;
//             p.z = 0; // On the ground (z=0)
//             line_strip.points.push_back(p);
//         }

//         // Publish the thicker line strip
//         marker_pub.publish(line_strip);

//         // Additional line strip for the trajectory (highlighter effect)
//         visualization_msgs::Marker highlight_strip;
//         highlight_strip.header.frame_id = "hero";
//         highlight_strip.header.stamp = ros::Time::now();
//         highlight_strip.ns = "trajectory_highlighter";
//         highlight_strip.action = visualization_msgs::Marker::ADD;
//         highlight_strip.pose.orientation.w = 1.0;

//         highlight_strip.id = 2;
//         highlight_strip.type = visualization_msgs::Marker::LINE_STRIP;
//         highlight_strip.scale.x = 1.8; // Thickness of the highlight line (wider)

//         // Line color (light green with lower opacity for the highlighter effect)
//         highlight_strip.color.r = 0.0;
//         highlight_strip.color.g = 1.0;
//         highlight_strip.color.b = 0.0;
//         highlight_strip.color.a = 0.05; // Lower opacity for highlighter effect

//         // Populate the points in the highlight line with z fixed to 0
//         for (const auto& point : trajectory.points) {
//             geometry_msgs::Point p;
//             p.x = point.x;
//             p.y = point.y;
//             p.z = 0; // Force z to be 0, fixing the line to the ground
//             highlight_strip.points.push_back(p);
//         }

//         // Publish the highlight line strip
//         marker_pub.publish(highlight_strip);

//         // Circles (spheres) at each point with filled color and outline
//         int point_id = 1000; // Start ID for the points
//         for (const auto& point : trajectory.points) {
//             // Sphere for the fill
//             visualization_msgs::Marker circle_fill;
//             circle_fill.header.frame_id = "hero";
//             circle_fill.header.stamp = ros::Time::now();
//             circle_fill.ns = "trajectory_points_fill";
//             circle_fill.action = visualization_msgs::Marker::ADD;
//             circle_fill.pose.orientation.w = 1.0;

//             circle_fill.id = point_id++;
//             circle_fill.type = visualization_msgs::Marker::SPHERE; // Use SPHERE for the filled circle
//             circle_fill.scale.x = 2.0; // Diameter of the circle
//             circle_fill.scale.y = 2.0;
//             circle_fill.scale.z = 0.01; // Very thin, almost flat on the ground

//             // Fill color (green with lower opacity)
//             circle_fill.color.r = 0.0;
//             circle_fill.color.g = 1.0;
//             circle_fill.color.b = 0.0;
//             circle_fill.color.a = 0.3; // Semi-transparent

//             // Set the position of the circle
//             circle_fill.pose.position.x = point.x;
//             circle_fill.pose.position.y = point.y;
//             circle_fill.pose.position.z = 0; // On the ground (z=0)

//             // Publish the filled circle
//             marker_pub.publish(circle_fill);

//             // Outline for the circle
//             visualization_msgs::Marker circle_outline;
//             circle_outline.header.frame_id = "hero";
//             circle_outline.header.stamp = ros::Time::now();
//             circle_outline.ns = "trajectory_points_outline";
//             circle_outline.action = visualization_msgs::Marker::ADD;
//             circle_outline.pose.orientation.w = 1.0;

//             circle_outline.id = point_id++;
//             circle_outline.type = visualization_msgs::Marker::LINE_STRIP; // Use LINE_STRIP for the circle outline
//             circle_outline.scale.x = 0.02; // Thickness of the outline
//             circle_outline.lifetime = ros::Duration(1.0);

//             // Outline color (dark green with full opacity)
//             circle_outline.color.r = 0.0;
//             circle_outline.color.g = 0.5; // Dark green
//             circle_outline.color.b = 0.0;
//             circle_outline.color.a = 1.0; // Full opacity

//             // Generate points for the circle outline
//             int num_points = 36; // Number of points in the circle
//             double radius = 1.0;
//             for (int i = 0; i <= num_points; ++i) {
//                 double angle = 2 * M_PI * i / num_points;
//                 geometry_msgs::Point p;
//                 p.x = point.x + radius * cos(angle);
//                 p.y = point.y + radius * sin(angle);
//                 p.z = 0; // On the ground (z=0)
//                 circle_outline.points.push_back(p);
//             }

//             // Publish the circle outline
//             marker_pub.publish(circle_outline);
//         }
//     }

//     // Function to predict trajectory in vehicle coordinates and then convert to ENU coordinates
//     behavior_trajectory_planner::TrajectoryArray predict_trajectory_ctrv(double x0, double y0, double theta0, double v0, double omega0, double T, double dt) {
//         // For leaving the parking area - 얼추 인덱스 물어보기
//         if (current_point_index > 7 && current_point_index < 40) {
//             v0 = 2.3; // Regard as 10km/h
//         }

//         // 필요하다면 lane change 상황에서도 위와 같은 작업이 필요할 수 있음
        
//         // Set Constraint velocity
//         if (v0 < 1.5) {
//             v0 = 1.5;
//         }

//         behavior_trajectory_planner::TrajectoryArray trajectory;

//         double max_yaw_rate = M_PI / 1.8 / T; // Maximum yaw rate that corresponds to pi/3.5 rad in 3 seconds
//         omega0 = std::clamp(omega0, -max_yaw_rate, max_yaw_rate); // Clamp omega0 to be within the range [-max_yaw_rate, max_yaw_rate]

//         double t = 0.0;

//         while (t <= T) {
//             behavior_trajectory_planner::TrajectoryPoint point;

//             double delta_theta = omega0 * t; // Yaw change in time t
//             double theta_t = theta0 + delta_theta; // Absolute yaw in ENU coordinates

//             double delta_x, delta_y;

//             if (fabs(omega0) > 0.2) { // If the yaw rate is significant, use a circular motion model
//                 delta_x = (v0 / omega0) * (sin(theta_t) - sin(theta0));
//                 delta_y = (v0 / omega0) * (-cos(theta_t) + cos(theta0));
//             } else { // If the yaw rate is small, approximate with a straight line
//                 delta_x = v0 * t * cos(theta0);
//                 delta_y = v0 * t * sin(theta0);
//             }

//             point.x = x0 + delta_x; // Convert to global ENU coordinates
//             point.y = y0 + delta_y; // Convert to global ENU coordinates
//             point.yaw = theta_t; // Yaw in global ENU coordinates
//             point.time = t;

//             trajectory.points.push_back(point);
//             t += dt;
//         }

//         return trajectory;
//     }

//     int get_road_option_from_csv(const std::string& path, int index) {
//         std::ifstream file(path);
//         if (!file.is_open()) {
//             ROS_ERROR("Failed to open file: %s", path.c_str());
//             return SHOULDER;
//         }

//         std::string line;
//         int current_index = SHOULDER;

//         while (std::getline(file, line)) {
//             if (current_index == index) {
//                 std::stringstream ss(line);
//                 std::string token;
//                 int column_count = 0;

//                 while (std::getline(ss, token, ',')) {
//                     if (column_count == 2) {
//                         file.close();
//                         return std::stoi(token);
//                     }
//                     column_count++;
//                 }
//             }
//             current_index++;
//         }

//         file.close();
//         ROS_ERROR("Index %d out of range in file: %s", index, path.c_str());
//         return SHOULDER;
//     }

//     void run() {
//         ros::Rate rate(20);

//         while (ros::ok()) {
//             auto start_time = std::chrono::high_resolution_clock::now();

//             std::string previous_trajectory = current_trajectory_path;

//             if (lane_change_done) {
//                 in_lane_change = false;
//                 path_index++;
//             }

            

//             if ((path_command == LANE_CHANGE) || in_lane_change) {
//                 ROS_INFO("Path Command : Lane Change");
//                 in_lane_change = true;
//                 current_trajectory_path = alternative_paths[path_index];
//                 if (path_index >= alternative_paths.size()) {
//                     path_index = alternative_paths.size() - 1;
//                 }
//             } else if ((path_command == AVOIDANCE) || local_planning) {
//                 ROS_INFO("Path Command : %f", path_command);
//                 ROS_INFO("Local planning : %s", local_planning);
//                 local_planning = true; /////////////// true ////////////asdfasdfasdfasdfasdfadsfsd
//                 current_trajectory_path = "/home/ailab/catkin_ws/src/carla_ros/path/global_path.csv";
//             } else {
//                 ROS_INFO("Path Command : Lane Following");
//                 current_trajectory_path = "/home/ailab/catkin_ws/src/carla_ros/path/global_path.csv";
//             }

//             std_msgs::String trajectory_msg;
//             trajectory_msg.data = current_trajectory_path;
//             trajectory_pub.publish(trajectory_msg);
            
//             int current_road_option = SHOULDER;
//             if (current_trajectory_path == "/home/ailab/catkin_ws/src/carla_ros/path/global_path.csv") {
//                 current_road_option = get_road_option_from_csv(current_trajectory_path, current_point_index);
//             }
//             std_msgs::Int32 road_option_msg;
//             road_option_msg.data = current_road_option;
//             current_road_option_pub.publish(road_option_msg);

//             std_msgs::Bool local_planning_msg;
//             local_planning_msg.data = local_planning;
//             local_planning_pub.publish(local_planning_msg);
            

//             float time_horizon = 3.0;
//             if (current_road_option == TURN) {
//                 time_horizon = 2.0;
//             } else {
//                 time_horizon = 3.0;
//             }

//             // Predict the trajectory in the ego frame
//             behavior_trajectory_planner::TrajectoryArray ego_trajectory = predict_trajectory_ctrv(0, 0, 0, speed, yaw_rate, time_horizon, 0.2);

//             // Calculate and add circles to the ego trajectory for visualization
//             for (const auto& point : ego_trajectory.points) {
//                 Eigen::Vector2d point_position(point.x, point.y);
//                 vector<Eigen::Vector2d> circles = calculate_circles(point_position, roll, pitch, point.yaw, 1.85, 4.72, 0.1);
                
//                 for (const auto& circle_pos : circles) {
//                     behavior_trajectory_planner::TrajectoryPoint circle_point;
//                     circle_point.x = circle_pos.x();
//                     circle_point.y = circle_pos.y();
//                     circle_point.yaw = point.yaw;
//                     circle_point.time = point.time; // keep the same time for synchronization
//                     ego_trajectory.points.push_back(circle_point);
//                 }
//             }

//             // Visualize the modified ego trajectory
//             visualize_trajectory(ego_trajectory);

//             // Predict and publish the ego trajectory in the global frame
//             behavior_trajectory_planner::TrajectoryArray ego_trajectory_enu = predict_trajectory_ctrv(x, y, yaw, speed, yaw_rate, time_horizon, 0.2);
//             ego_trajectory_pub.publish(ego_trajectory_enu);

//             if (previous_trajectory != current_trajectory_path) {
//                 ROS_INFO("Trajectory Path updated to: %s", current_trajectory_path.c_str());
//             }

//             auto end_time = std::chrono::high_resolution_clock::now();
//             std::chrono::duration<double> elapsed = end_time - start_time;
//             ROS_INFO("Loop execution time: %f seconds", elapsed.count());

//             ros::spinOnce();
//             rate.sleep();
//         }
//     }

// private:
//     ros::Subscriber class_sub;
//     ros::Subscriber lane_change_done_sub;
//     ros::Subscriber local_planning_sub;
//     ros::Subscriber current_point_index_sub;
//     ros::Subscriber enu_sub;
//     ros::Subscriber speed_sub;
//     ros::Subscriber yaw_rate_sub;
//     ros::Subscriber gt_position_sub;
    
//     ros::Publisher trajectory_pub;
//     ros::Publisher ego_trajectory_pub;
//     ros::Publisher current_road_option_pub;
//     ros::Publisher local_planning_pub;
//     ros::Publisher marker_pub;

//     int path_command;
//     std::string current_trajectory_path;
//     std::vector<std::string> alternative_paths;
//     int path_index;
//     int current_point_index;
//     bool in_lane_change;
//     bool lane_change_done;
//     bool local_planning;
//     double x, y, yaw, pitch, roll, yaw_rate, speed;
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "behavior_trajectory_planner");
//     BehaviorTrajectoryPlanner planner;
//     planner.run();
//     return 0;
// }