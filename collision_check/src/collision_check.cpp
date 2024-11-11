#include <iostream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <derived_object_msgs/ObjectArray.h>
#include <derived_object_msgs/Object.h>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h> // For converting quaternion to yaw
#include <std_msgs/Int32.h>
#include <collision_check/TrajectoryArray.h>
#include <collision_check/TrajectoryPoint.h>
#include <collision_check/TrackingArray.h> ///// 
#include <collision_check/Tracking.h> /////
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
    double roll;
    double yaw;
    double pitch;
};

vector<Vehicle> vehicles;

class CollisionChecker {
public:
    CollisionChecker() : current_road_option(0) {
        ros::NodeHandle nh;
        object_info_sub = nh.subscribe("/object_tracking", 1000, &CollisionChecker::object_info_callback, this);
        ego_trajectory_sub = nh.subscribe("/ego_trajectory", 10, &CollisionChecker::ego_trajectory_callback, this);
        current_road_option_sub = nh.subscribe("current_road_option", 10, &CollisionChecker::road_option_callback, this);
        yaw_rate_sub = nh.subscribe("carla/hero/IMU", 1000, &CollisionChecker::imu_callback, this);

        collision_risk_pub = nh.advertise<std_msgs::Int32>("collision_risk", 10);
        velocity_increase_pub = nh.advertise<std_msgs::Int32>("velocity_increase", 10);
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("surround_motion_prediction", 10);  // Use MarkerArray for RViz visualization

        // New publisher for predicted trajectories
        predicted_trajectories_pub = nh.advertise<collision_check::TrajectoryArray>("predicted_trajectories", 10);
    }

    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
        // Convert quaternion to roll, pitch, yaw
        tf::Quaternion quat(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf::Matrix3x3 m(quat);
        m.getRPY(ego_roll, ego_pitch, ego_yaw);
        
        // // Debugging: Print the roll, pitch, yaw values
        // ROS_INFO("IMU Callback - Roll: %f, Pitch: %f, Yaw: %f", ego_roll, ego_pitch, ego_yaw);
    }

    void road_option_callback(const std_msgs::Int32::ConstPtr& msg) {
        current_road_option = msg->data;
    }

    void object_info_callback(const collision_check::TrackingArray::ConstPtr& msg) {
        vehicles.clear(); // Clear the previous vehicles list
        
        for (const auto& track : msg->tracks) {
            Vehicle vehicle;

            // Extract position
            vehicle.position = Vector2d(track.x, track.y);

            // Extract yaw directly from the message
            vehicle.yaw = track.yaw;

            // Roll and pitch are set to 0
            vehicle.roll = 0.0;
            vehicle.pitch = 0.0;

            // Extract velocity
            vehicle.velocity = Vector2d(track.vx, track.vy);

            // Store the vehicle in the list
            vehicles.push_back(vehicle);
        }
    }

    // Callback function for /ego_trajectory topic
    void ego_trajectory_callback(const collision_check::TrajectoryArray::ConstPtr& msg) {
        ego_trajectory = *msg; // Update the ego trajectory
    }
    
    collision_check::TrajectoryArray predict_motion(const Vehicle& vehicle, double time_horizon, double dt, double roll, double pitch, double yaw) {
        collision_check::TrajectoryArray trajectory_array;
        Vector2d current_position = vehicle.position;

        // Predict position at each time step
        for (double t = 0; t <= time_horizon; t += dt) {
            collision_check::TrajectoryPoint point;

            // Calculate local deltas
            double del_x = vehicle.velocity.x() * t;
            double del_y = vehicle.velocity.y() * t;

            // Update the point position in global coordinates
            point.x = current_position.x() + del_x;
            point.y = current_position.y() + del_y;

            // Update the yaw (assuming constant yaw rate)
            point.yaw = yaw;
            point.time = t;
            trajectory_array.points.push_back(point);
        }

        return trajectory_array;
    }

    void visualize_surround_trajectory(const collision_check::TrajectoryArray& trajectory, int vehicle_id, visualization_msgs::MarkerArray& marker_array) {
        // Line strip for the surround vehicle's trajectory (thin, high opacity)
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "hero";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "surround_trajectory_lines";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.lifetime = ros::Duration(1.0);

        line_strip.id = vehicle_id;  // Unique ID for each vehicle trajectory
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = 0.15; // Thickness of the line (thin)

        // Line color 
        line_strip.color.r = 0.5;
        line_strip.color.g = 0.8;
        line_strip.color.b = 1.0;
        line_strip.color.a = 0.8; // Higher opacity

        // Populate the points in the line
        for (const auto& point : trajectory.points) {
            Vector2d point_position(point.x, point.y);
            Vector2d ego_position(ego_trajectory.points[0].x, ego_trajectory.points[0].y);

            // Translate ENU to Ego vehicle coordinate frame
            Vector2d relative_position = point_position - ego_position;

            // Rotate to align with Ego vehicle's orientation
            Matrix3d R;
            R = AngleAxisd(ego_yaw, Vector3d::UnitZ()) *
                AngleAxisd(ego_pitch, Vector3d::UnitY()) *
                AngleAxisd(ego_roll, Vector3d::UnitX());

            Vector3d transformed_position = R.inverse() * Vector3d(relative_position.x(), relative_position.y(), 0);

            geometry_msgs::Point p;
            p.x = transformed_position.x();
            p.y = transformed_position.y();
            p.z = 0; // On the ground (z=0)
            line_strip.points.push_back(p);
        }

        // Add the line strip marker to the array
        marker_array.markers.push_back(line_strip);

        // Circles (spheres) at each point with filled color and outline
        int point_id = vehicle_id * 1000; // Start ID for the points, unique per vehicle
        for (const auto& point : trajectory.points) {
            Vector2d point_position(point.x, point.y);
            Vector2d ego_position(ego_trajectory.points[0].x, ego_trajectory.points[0].y);

            // Translate ENU to Ego vehicle coordinate frame
            Vector2d relative_position = point_position - ego_position;

            // Rotate to align with Ego vehicle's orientation
            Matrix3d R;
            R = AngleAxisd(ego_yaw, Vector3d::UnitZ()) *
                AngleAxisd(ego_pitch, Vector3d::UnitY()) *
                AngleAxisd(ego_roll, Vector3d::UnitX());

            Vector3d transformed_position = R.inverse() * Vector3d(relative_position.x(), relative_position.y(), 0);

            // Filled circle (SPHERE) for the inner part
            visualization_msgs::Marker circle_fill;
            circle_fill.header.frame_id = "hero";
            circle_fill.header.stamp = ros::Time::now();
            circle_fill.ns = "surround_trajectory_points_fill";
            circle_fill.action = visualization_msgs::Marker::ADD;
            circle_fill.pose.orientation.w = 1.0;
            circle_fill.lifetime = ros::Duration(1.0);

            circle_fill.id = point_id++;
            circle_fill.type = visualization_msgs::Marker::SPHERE; // Use SPHERE for the filled circle
            circle_fill.scale.x = 2.0; // Diameter of the circle
            circle_fill.scale.y = 2.0;
            circle_fill.scale.z = 0.01; // Very thin, almost flat on the ground

            // Fill color (light blue with lower opacity)
            circle_fill.color.r = 0.5;
            circle_fill.color.g = 0.8;
            circle_fill.color.b = 1.0; 
            circle_fill.color.a = 0.2; // Semi-transparent

            // Set the position of the circle
            circle_fill.pose.position.x = transformed_position.x();
            circle_fill.pose.position.y = transformed_position.y();
            circle_fill.pose.position.z = 0; // On the ground (z=0)

            // Add the filled circle to the marker array
            marker_array.markers.push_back(circle_fill);

            // Outline for the circle
            visualization_msgs::Marker circle_outline;
            circle_outline.header.frame_id = "hero";
            circle_outline.header.stamp = ros::Time::now();
            circle_outline.ns = "surround_trajectory_points_outline";
            circle_outline.action = visualization_msgs::Marker::ADD;
            circle_outline.pose.orientation.w = 1.0;
            circle_outline.lifetime = ros::Duration(1.0);

            circle_outline.id = point_id++;
            circle_outline.type = visualization_msgs::Marker::LINE_STRIP; // Use LINE_STRIP for the circle outline
            circle_outline.scale.x = 0.02; // Thickness of the circle outline

            // Outline color (darker blue)
            circle_outline.color.r = 0.4;
            circle_outline.color.g = 0.6;
            circle_outline.color.b = 0.9;
            circle_outline.color.a = 1.0; // Full opacity

            // Generate points for the circle outline
            int num_points = 36; // Number of points in the circle
            double radius = 1.0;
            for (int i = 0; i <= num_points; ++i) {
                double angle = 2 * M_PI * i / num_points;
                geometry_msgs::Point p;
                p.x = transformed_position.x() + radius * cos(angle);
                p.y = transformed_position.y() + radius * sin(angle);
                p.z = 0; // On the ground (z=0)
                circle_outline.points.push_back(p);
            }

            // Add the circle outline to the marker array
            marker_array.markers.push_back(circle_outline);
        }
    }

    vector<Vector2d> calculate_circles_surround(const Vector2d& position, double yaw, double vehicle_width, double vehicle_length, double radius) {
        vector<Vector2d> circles; // vector for 3 circles points

        // Vehicle's center position (ENU)
        Vector2d center(position.x(), position.y());

        // Define the rotation matrix using yaw (2D rotation)
        Rotation2Dd rotation(yaw);

        // Define points in the vehicle's local frame
        Vector2d front_point(2.0, 0.0);  // x: +2, y: 0
        Vector2d center_point(0.0, 0.0); // x: 0, y: 0
        Vector2d rear_point(-2.0, 0.0);  // x: -2, y: 0

        // Rotate the points to the global frame using the yaw value
        Vector2d front_global = rotation * front_point;
        Vector2d center_global = rotation * center_point;
        Vector2d rear_global = rotation * rear_point;

        // Translate the points to the vehicle's global position
        Vector2d front_circle = center + front_global;
        Vector2d center_circle = center + center_global;
        Vector2d rear_circle = center + rear_global;

        // Store the circle positions in the vector
        circles.push_back(front_circle);
        circles.push_back(center_circle);
        circles.push_back(rear_circle);

        return circles;
    }

    vector<Vector2d> calculate_circles(const Vector2d& position, double roll, double pitch, double yaw, double vehicle_width, double vehicle_length, double radius) {
        vector<Vector2d> circles; // [x,y] 3개를 가지고 있는 벡터 리스트

        // Vehicle's center position (ENU)
        Vector2d center(position.x(), position.y());

        // Define the transformation matrix using roll, pitch, yaw (RPY)
        Matrix3d R;
        R = AngleAxisd(yaw, Vector3d::UnitZ()) * /////////////////////////////////afsfafd/////////dfasdf////////////////asdfadfadfa
            AngleAxisd(pitch, Vector3d::UnitY()) *
            AngleAxisd(roll, Vector3d::UnitX());

        // Inverse of the transformation matrix
        Matrix3d R_inv = R.inverse();

        // Define points in the vehicle's local frame (assuming z=0 for a 2D transformation)
        Vector3d front_point(2.0, 0.0, 0.0); // x: +1, y: 0
        Vector3d center_point(0.0, 0.0, 0.0); // x: 0, y: 0
        Vector3d rear_point(-2.0, 0.0, 0.0); // x: -1, y: 0

        // Transform the points to the global frame using the inverse transformation
        Vector3d front_global = R_inv * front_point;
        Vector3d center_global = R_inv * center_point;
        Vector3d rear_global = R_inv * rear_point;

        // Convert the 3D points to 2D by ignoring the z component
        Vector2d front_circle = center + Vector2d(front_global.x(), front_global.y());
        Vector2d center_circle = center + Vector2d(center_global.x(), center_global.y());
        Vector2d rear_circle = center + Vector2d(rear_global.x(), rear_global.y());

        // Store the circle positions in the vector
        circles.push_back(front_circle);
        circles.push_back(center_circle);
        circles.push_back(rear_circle);

        return circles;
    }

    void run() {
        ros::Rate rate(20); // 20 Hz
        double time_horizon = 3.0; // Default time horizon
        double dt = 0.2; // 0.2 seconds per step

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
                time_horizon = 2.0; // time horizon at turn situation
            } 
            else {
                time_horizon = 3.0;
            }

            // For Debugging
            ROS_INFO("Number of vehicles : %lu", vehicles.size());
            ROS_INFO("Current road option : %d", current_road_option);

            double min_collision_time = 999; // Initialize to 999 at the start of each cycle
            int vehicles_within_range = 0; // Counter for vehicles within 35 meters
            int should_increase_velocity = 0; // To determine if ego velocity should be increased for intersection situation

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

                bool collision_with_behind_vehicle = false;

                for (const auto& vehicle : vehicles) {
                    // Calculate the distance between the ego vehicle and the current vehicle
                    Vector2d ego_position(ego_trajectory.points[0].x, ego_trajectory.points[0].y);
                    Vector2d vehicle_position = vehicle.position;

                    double distance_to_ego = (ego_position - vehicle_position).norm();

                    // Transform the vehicle's position from the ego vehicle's frame
                    Vector2d relative_position = vehicle_position - ego_position;

                    // Rotate to align with Ego vehicle's orientation
                    Matrix3d R;
                    R = AngleAxisd(ego_yaw, Vector3d::UnitZ()) *
                        AngleAxisd(ego_pitch, Vector3d::UnitY()) *
                        AngleAxisd(ego_roll, Vector3d::UnitX());

                    Vector3d transformed_position = R.inverse() * Vector3d(relative_position.x(), relative_position.y(), 0);

                    // Collision check about only distance < 50 on basis of ego position
                    if (distance_to_ego > 50.0) {
                        continue; // Skip this vehicle if it's outside the 50 meter range
                    }
                    if ((transformed_position.x() < 5) && (transformed_position.x() > -5)) {
                        continue;
                    }

                    // Collision check about only abs(theta)<60 on basis of ego vehicle frame
                    double angle_to_vehicle = atan2(transformed_position.y(), transformed_position.x()) * 180 / M_PI;
                    if (abs(angle_to_vehicle) >= 50.0 && abs(angle_to_vehicle) <= 120) {
                        continue;
                    }
                    if ((abs(angle_to_vehicle) >= 165.0)) {
                        continue;
                    }

                    vehicles_within_range++; // Increment the counter for vehicles within range

                    // Predict positions over the time horizon using the CV model
                    collision_check::TrajectoryArray surround_trajectory = predict_motion(vehicle, time_horizon, dt, 0, 0, vehicle.yaw);

                    // Ensure that surround_trajectory has the same number of points as ego_trajectory
                    if (surround_trajectory.points.size() != ego_trajectory.points.size()) {
                        ROS_WARN("Size mismatch between ego and surround trajectory points.");
                        continue;  // Skip processing this vehicle if there's a size mismatch
                    }

                    size_t num_points = ego_trajectory.points.size();

                    for (size_t idx = 0; idx < num_points; ++idx) { // 15 times iteration
                        const auto& ego_point = ego_trajectory.points[idx]; // ENU frame
                        const auto& surround_point = surround_trajectory.points[idx]; // ENU frame

                        // Convert ego_point and surround_point to Vector2d
                        Vector2d ego_position(ego_point.x, ego_point.y);
                        Vector2d surround_position(surround_point.x, surround_point.y);

                        // Calculate the positions of the 3 circles for ego and surround vehicles
                        vector<Vector2d> ego_circles  = calculate_circles(ego_position, ego_roll, ego_pitch, ego_yaw, vehicle_width, vehicle_length, radius);
                        vector<Vector2d> surround_circles;

                        if (vehicle.velocity[0] < 5.0 && vehicle.velocity[1] < 5.0) {
                            surround_circles = calculate_circles_surround(surround_position, ego_yaw, vehicle_width, vehicle_length, radius);
                        } else {
                            surround_circles = calculate_circles_surround(surround_position, vehicle.yaw, vehicle_width, vehicle_length, radius);
                        }

                        // TODO : 여기서 surround_trajectory 안에 계산한 surround_circles 3개의 원의 좌표를 추가
                        for (const auto& circle_pos : surround_circles) { // 3 times
                            collision_check::TrajectoryPoint circle_point;
                            circle_point.x = circle_pos.x();
                            circle_point.y = circle_pos.y();
                            circle_point.yaw = 0.0; 
                            circle_point.time = 0.0; 

                            surround_trajectory.points.push_back(circle_point);
                        }

                      
                        // Check for collision between any of the circles
                        bool collision_detected = false;
                        for (const auto& ego_circle : ego_circles) {
                            for (const auto& surround_circle : surround_circles) {
                                double distance = (ego_circle - surround_circle).norm();
                                if (distance < 1.9) { // Potential collision : 1.9m = 2 * radius
                                    ROS_INFO("min_collision_time : %f seconds", min_collision_time);
                                    ROS_INFO("ego_point.time : %f seconds", ego_point.time);

                                    if (ego_point.time < 3.0) {
                                        if ((transformed_position.x() > 25.0) && (abs(angle_to_vehicle) <= 20)) {
                                            should_increase_velocity = 1;
                                        }
                                    }
                                    min_collision_time = std::min(min_collision_time, ego_point.time);
                                    collision_detected = true;
                                    break;
                                }
                            }
                            if (collision_detected) break;
                        }
                    }

                    // Visualize the predicted trajectory for each surrounding vehicle using MarkerArray
                    visualize_surround_trajectory(surround_trajectory, vehicle_id++, marker_array);
                }

                // Publish all markers at once
                marker_pub.publish(marker_array);

                ROS_INFO("Min TTC : %f seconds", min_collision_time);
                ROS_INFO("Number of vehicles within 45 meters: %d", vehicles_within_range); // Print the number of vehicles within range
                
                // Clear the vehicle list for the next cycle
                vehicles.clear();
            }

            // Publish Collision risk: TTC threshold 3.0 for stopping and 1.5 for AEB
            int collision_risk = SAFE;
            float TTC = 1.5;

            if (current_road_option == TURN) {
                TTC = 1.5;
            } else if (current_road_option == TWO_LANE) {
                TTC = 2.5;
            }

            if (min_collision_time < TTC) {
                collision_risk = AEB;
            } else if (min_collision_time < 3.0) {
                collision_risk = STOP;
            }

            std_msgs::Int32 collision_risk_msg;
            collision_risk_msg.data = collision_risk;
            collision_risk_pub.publish(collision_risk_msg);

            // Publish the velocity increase bool
            std_msgs::Int32 velocity_increase_msg;
            velocity_increase_msg.data = should_increase_velocity;
            velocity_increase_pub.publish(velocity_increase_msg);

            // End time measurement
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end_time - start_time;
            ROS_INFO("Loop execution time: %f seconds", elapsed.count());
            ROS_INFO("Ego position: x = %f, y = %f, yaw = %f", ego_trajectory.points[0].x, ego_trajectory.points[0].y, ego_trajectory.points[0].yaw);

            rate.sleep();
        }
    }

private:
    ros::Subscriber object_info_sub;
    ros::Subscriber ego_trajectory_sub;
    ros::Subscriber current_road_option_sub;
    ros::Subscriber yaw_rate_sub;
    ros::Publisher collision_risk_pub;
    ros::Publisher marker_pub;
    ros::Publisher predicted_trajectories_pub;
    ros::Publisher velocity_increase_pub;

    collision_check::TrajectoryArray ego_trajectory;
    int current_road_option; // Variable to store the current road option
    double ego_roll;
    double ego_pitch;
    double ego_yaw;

    double quaternion_to_yaw(const geometry_msgs::Quaternion& q) {
        tf::Quaternion quat(q.x, q.y, q.z, q.w);
        tf::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "collision_check_node");
    CollisionChecker collision_checker;
    collision_checker.run();

    return 0;
}