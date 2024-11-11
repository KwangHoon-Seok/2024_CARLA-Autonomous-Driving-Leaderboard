#include <iostream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <derived_object_msgs/ObjectArray.h>
#include <derived_object_msgs/Object.h>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <tf/tf.h> // For converting quaternion to yaw
#include <std_msgs/Int32.h>
#include <collision_check/TrajectoryArray.h>
#include <collision_check/TrajectoryPoint.h>
#include <chrono> // For measuring time
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h> // For RViz visualization with MarkerArray

using namespace std;
using namespace Eigen;

const int AEB = 100;
const int STOP = 101;
const int SAFE = 102;

// Road Option
const int SHOULDER = 0; // 20km/h
const int TWO_LANE = 1; // 40km/h
const int STRAIGHT = 2; // 25km/h
const int TURN = 3; // 15km/h

struct Vehicle {
    Vector2d position; // [e, n]
    Vector2d velocity; // [vx, vy]
    double yaw;
};

vector<Vehicle> vehicles;

class CollisionChecker {
public:
    CollisionChecker() : current_road_option(STRAIGHT) {
        ros::NodeHandle nh;
        object_info_sub = nh.subscribe("/carla/hero/Objects", 1000, &CollisionChecker::object_info_callback, this);
        ego_trajectory_sub = nh.subscribe("/ego_trajectory", 10, &CollisionChecker::ego_trajectory_callback, this);
        current_road_option_sub = nh.subscribe("current_road_option", 10, &CollisionChecker::road_option_callback, this);
        collision_risk_pub = nh.advertise<std_msgs::Int32>("collision_risk", 10);
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("surround_motion_prediction", 10);  // Use MarkerArray for RViz visualization
        predicted_trajectories_pub = nh.advertise<collision_check::TrajectoryArray>("predicted_trajectories", 10);
    }

    void road_option_callback(const std_msgs::Int32::ConstPtr& msg) {
        current_road_option = msg->data;
    }

    vector<Vector2d> calculate_circles_with_rotation(const Vector2d& position, double roll, double pitch, double yaw, double vehicle_width, double vehicle_length, double radius) {
        vector<Vector2d> circles; // [x, y] 3개를 가지고 있는 벡터 리스트

        // Vehicle's center position
        Vector2d center(position.x(), position.y());

        // Calculate the rotation matrix using roll, pitch, and yaw
        Matrix3d rotation_matrix;
        rotation_matrix = AngleAxisd(roll, Vector3d::UnitX())
                        * AngleAxisd(pitch, Vector3d::UnitY())
                        * AngleAxisd(yaw, Vector3d::UnitZ());

        // Convert 2D offsets to 3D to apply the full rotation matrix
        Vector3d front_offset_3d(vehicle_length / 2.0, 0, 0);
        Vector3d rear_offset_3d(-vehicle_length / 2.0, 0, 0);

        // Rotate the offsets
        Vector3d rotated_front_offset = rotation_matrix * front_offset_3d;
        Vector3d rotated_rear_offset = rotation_matrix * rear_offset_3d;

        // Project back to 2D after rotation
        Vector2d front_offset(rotated_front_offset.x(), rotated_front_offset.y());
        Vector2d rear_offset(rotated_rear_offset.x(), rotated_rear_offset.y());

        // Calculate the center, front, and rear circle positions
        circles.push_back(center + front_offset);
        circles.push_back(center);
        circles.push_back(center + rear_offset);

        return circles;
    }

    void object_info_callback(const derived_object_msgs::ObjectArray::ConstPtr& msg) {
        vehicles.clear(); // Clear the previous vehicles list

        for (const auto& obj : msg->objects) {
            // Check if the object is classified as a vehicle (classification == 6)
            if (obj.classification == 6) {
                Vehicle vehicle;

                // Extract position
                vehicle.position = Vector2d(obj.pose.position.x, obj.pose.position.y);

                // Extract yaw from orientation (quaternion to yaw)
                vehicle.yaw = quaternion_to_yaw(obj.pose.orientation);

                // Extract velocity (assume twist is in m/s)
                vehicle.velocity = Vector2d(obj.twist.linear.x, obj.twist.linear.y);

                // Store the vehicle in the list
                vehicles.push_back(vehicle);
            }
        }
    }

    // Callback function for /ego_trajectory topic
    void ego_trajectory_callback(const collision_check::TrajectoryArray::ConstPtr& msg) {
        ego_trajectory = *msg; // Update the ego trajectory
    }

    // Motion Prediction using Constant Velocity(CV) Model
    collision_check::TrajectoryArray predict_motion(const Vehicle& vehicle, double time_horizon, double dt) {
        collision_check::TrajectoryArray trajectory_array;
        Vector2d current_position = vehicle.position;
        double current_yaw = vehicle.yaw; // current surround vehicle yaw

        // Predict position at each time step
        for (double t = 0; t <= time_horizon; t += dt) {
            collision_check::TrajectoryPoint point;

            // Convert vx, vy to ENU Coordinate
            double vx_global = vehicle.velocity.x() * cos(current_yaw) - vehicle.velocity.y() * sin(current_yaw);
            double vy_global = vehicle.velocity.x() * sin(current_yaw) + vehicle.velocity.y() * cos(current_yaw);

            point.x = current_position.x() + vx_global * t;
            point.y = current_position.y() + vy_global * t;
            point.yaw = current_yaw;  // Assuming constant yaw
            point.time = t;
            trajectory_array.points.push_back(point);
        }

        return trajectory_array;
    }

    // Function to visualize the predicted trajectory in RViz
    void visualize_surround_trajectory(const collision_check::TrajectoryArray& trajectory, int vehicle_id, visualization_msgs::MarkerArray& marker_array) {
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "hero"; // or "base_link", depending on your setup
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "surround_trajectory_lines";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;

        line_strip.id = vehicle_id;  // Unique ID for each vehicle

        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        // LINE_STRIP/LINE_LIST markers use only the x, y, and z fields of points.
        line_strip.scale.x = 1.8;  // Thickness of the line, set to vehicle width

        // Line color (e.g., green with full opacity)
        line_strip.color.r = 0.0;
        line_strip.color.g = 1.0;
        line_strip.color.b = 0.0;
        line_strip.color.a = 0.5;

        // Populate the points
        for (const auto& point : trajectory.points) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;  // Assuming you want to visualize it on the ground
            line_strip.points.push_back(p);
        }

        marker_array.markers.push_back(line_strip);  // Add the marker to the array
    }

    void run() {
        ros::Rate rate(20); // 20 Hz
        double time_horizon = 3.0; // Default time horizon
        double dt = 0.1; // 0.1 seconds per step

        // Vehicle dimensions
        double vehicle_width = 1.85;
        double vehicle_length = 4.72;
        double radius = 0.1; // 0.1 meter for each circle

        while (ros::ok()) {
            // Start time measurement
            auto start_time = std::chrono::high_resolution_clock::now();

            ros::spinOnce();

            // Adjust time_horizon based on current_road_option
            if (current_road_option == TURN) {
                time_horizon = 2.0;
            } else {
                time_horizon = 3.0;
            }

            // For Debugging
            ROS_INFO("Number of vehicles : %lu", vehicles.size());
            ROS_INFO("Current road option : %d", current_road_option);

            double min_collision_time = 999; // Initialize to 999 at the start of each cycle
            int vehicles_within_range = 0; // Counter for vehicles within 35 meters

            // Check if ego_trajectory is empty
            if (ego_trajectory.points.empty()) {
                ROS_WARN("Ego trajectory is empty.");
                continue;  // Skip processing if ego trajectory is empty
            }

            if (!vehicles.empty()) {
                int vehicle_id = 0;  // ID counter for markers
                visualization_msgs::MarkerArray marker_array;  // Create a MarkerArray for all markers

                // Get the ego vehicle's yaw angle
                double ego_yaw = ego_trajectory.points[2].yaw;

                for (const auto& vehicle : vehicles) {
                    // Calculate the distance between the ego vehicle and the current vehicle
                    Vector2d ego_position(ego_trajectory.points[0].x, ego_trajectory.points[0].y);
                    Vector2d vehicle_position = vehicle.position;

                    // Transform the vehicle's position to the ego vehicle's frame
                    Vector2d relative_position = vehicle_position - ego_position;
                    Eigen::Matrix2d rotation_matrix;
                    rotation_matrix << cos(ego_yaw), sin(ego_yaw),
                                    -sin(ego_yaw), cos(ego_yaw);
                    Vector2d transformed_position = rotation_matrix * relative_position;

                    // Check if the transformed x coordinate is positive (i.e., in front of the ego vehicle)
                    if (transformed_position.x() <= 0) {
                        continue; // Skip this vehicle if it's behind or exactly on the ego vehicle's x-axis
                    }

                    // Check if the vehicle is within 45 meters of the ego vehicle
                    double distance_to_ego = transformed_position.norm();
                    if (distance_to_ego > 45.0) {
                        continue; // Skip this vehicle if it's outside the 45 meter range
                    }

                    vehicles_within_range++; // Increment the counter for vehicles within range

                    // Predict positions over the time horizon using the CV model
                    collision_check::TrajectoryArray surround_trajectory = predict_motion(vehicle, time_horizon, dt);

                    // Ensure that surround_trajectory has the same number of points as ego_trajectory
                    if (surround_trajectory.points.size() != ego_trajectory.points.size()) {
                        ROS_WARN("Size mismatch between ego and surround trajectory points.");
                        continue;  // Skip processing this vehicle if there's a size mismatch
                    }

                    // Visualize the predicted trajectory for each surrounding vehicle using MarkerArray
                    visualize_surround_trajectory(surround_trajectory, vehicle_id++, marker_array);

                    size_t num_points = ego_trajectory.points.size();

                    for (size_t idx = 0; idx < num_points; ++idx) {
                        const auto& ego_point = ego_trajectory.points[idx];
                        const auto& surround_point = surround_trajectory.points[idx];

                        // Calculate the positions of the 3 circles for ego and surround vehicles
                        vector<Vector2d> ego_circles = calculate_circles_with_rotation(Vector2d(ego_point.x, ego_point.y), ego_point.roll, ego_point.pitch, ego_point.yaw, vehicle_width, vehicle_length, radius);
                        vector<Vector2d> surround_circles = calculate_circles_with_rotation(Vector2d(surround_point.x, surround_point.y), surround_point.roll, surround_point.pitch, surround_point.yaw, vehicle_width, vehicle_length, radius);

                        // Check for collision between any of the circles
                        bool collision_detected = false;
                        for (const auto& ego_circle : ego_circles) {
                            for (const auto& surround_circle : surround_circles) {
                                double distance = (ego_circle - surround_circle).norm();
                                if (distance < 1.5) { // Potential collision : 1.5m
                                    min_collision_time = std::min(min_collision_time, ego_point.time);
                                    collision_detected = true;
                                    break;
                                }
                            }
                            if (collision_detected) break;
                        }
                    }
                }

                // Publish all markers at once
                marker_pub.publish(marker_array);
                ROS_INFO("Min TTC : %f seconds", min_collision_time);
                ROS_INFO("Number of vehicles within 45 meters: %d", vehicles_within_range); // Print the number of vehicles within range
            }

            // Publish Collision risk: TTC threshold 3.0 for stopping and 1.5 for AEB
            int collision_risk = SAFE;
            if (min_collision_time < 1.5) {
                collision_risk = AEB;
            } else if (min_collision_time < 1.8) {
                collision_risk = STOP;
            }

            std_msgs::Int32 collision_risk_msg;
            collision_risk_msg.data = collision_risk;
            collision_risk_pub.publish(collision_risk_msg);

            // End time measurement
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end_time - start_time;
            ROS_INFO("Loop execution time: %f seconds", elapsed.count());
            ROS_INFO("Ego position: x = %f, y = %f", ego_position.x(), ego_position.y());

            rate.sleep();
        }
    }

private:
    ros::Subscriber object_info_sub;
    ros::Subscriber ego_trajectory_sub;
    ros::Subscriber current_road_option_sub;
    ros::Publisher collision_risk_pub;
    ros::Publisher marker_pub;
    ros::Publisher predicted_trajectories_pub;

    collision_check::TrajectoryArray ego_trajectory;
    int current_road_option; // Variable to store the current road option

    double quaternion_to_yaw(const geometry_msgs::Quaternion& q) {
        tf::Quaternion quat(q.x, q.y, q.z, q.w);
        tf::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "collision_check_node_test");

    CollisionChecker collision_checker;
    collision_checker.run();

    return 0;
}
