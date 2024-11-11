#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <velocity_planner/DetectionInfo.h>
#include <velocity_planner/DetectionInfoArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>

// Define constants
const int AEB = 100;
const int STOP = 101;
const int SAFE = 102;

const int SHOULDER = 0;
const int TWO_LANE = 1;
const int STRAIGHT = 2;
const int TURN = 3;

const int RED = 5;
const int STOP_SIGN = 6;
const int YELLOW = 7;

class VelocityPlanner {
public:
    VelocityPlanner() {
        ros::NodeHandle nh;

        // Initialize subscribers
        current_road_option_sub = nh.subscribe("/current_road_option", 10, &VelocityPlanner::road_option_callback, this);
        collision_risk_sub = nh.subscribe("/collision_risk", 10, &VelocityPlanner::collision_risk_callback, this);
        yolo_data_sub = nh.subscribe("/detection_with_distance", 10, &VelocityPlanner::yolo_data_callback, this);
        velocity_increase_sub = nh.subscribe("/velocity_increase", 10, &VelocityPlanner::velocity_increase_callback, this);
        current_index_sub = nh.subscribe("/current_point_index", 10, &VelocityPlanner::current_index_callback, this);
        
        // Publisher for reference velocity
        reference_velocity_pub = nh.advertise<std_msgs::Float32>("/reference_velocity", 10);
        // Publisher for AEB status visualization
        aeb_status_marker_pub = nh.advertise<visualization_msgs::Marker>("aeb_status_marker", 1);

        // Initialize variables
        collision_risk = SAFE;
        current_road_option = TURN;
        red_light_detected = false;
        human_detected = false;
        human_distance = 0.0;
        velocity_increased = 0;
        aeb_active = false;
        ignore_stop_sign = false;
        human_aeb_active = false;
        red_light_aeb_active = false;
        current_index = 0;
    }

    void velocity_increase_callback(const std_msgs::Int32::ConstPtr& msg) {
        velocity_increased = msg->data;
    }

    void road_option_callback(const std_msgs::Int32::ConstPtr& msg) {
        current_road_option = msg->data;
    }

    void collision_risk_callback(const std_msgs::Int32::ConstPtr& msg) {
        collision_risk = msg->data;
    }

    void current_index_callback(const std_msgs::Int32::ConstPtr& msg) {
        current_index = msg->data;
    }

    int check_index_ranges(int current_index) {
        std::vector<std::pair<int, int>> ranges = {
            {665, 690},
            {1495, 1515},
            {2450, 2482},
            {3125, 3160},
            {4015, 4030},
            {4685, 4700},
            {5537, 5555},
            {6840, 6865},
            {7303, 7320},
            {7980, 8005},
            {8550, 8580},
            {9215, 9235}
        };
        for (const auto& range : ranges) {
            if (current_index >= range.first && current_index <= range.second) {
                return 1;
            }
        }
        return 0;
    }

    int slow_down(int current_index) {
        std::vector<std::pair<int, int>> ranges = {
            {645, 674},
            {1475, 1504},
            {2440, 2469},
            {3110, 3139},
            {4085, 4014},
            {4655, 4684},
            {5507, 5536},
            {6810, 6839},
            {7273, 7302},
            {7950, 7979},
            {8520, 8549},
            {9185, 9214}
        };
        for (const auto& range : ranges) {
            if (current_index >= range.first && current_index <= range.second) {
                return 1;
            }
        }
        return 0;
    }

    void yolo_data_callback(const velocity_planner::DetectionInfoArray::ConstPtr& msg) {
        if (ignore_stop_sign) {
            return; // STOP_SIGN 무시 중일 때는 감지하지 않음
        }

        // Iterate through each detection in the array
        red_light_detected = false;
        human_detected = false;
        human_distance = 0.0;

        int index_in_range = check_index_ranges(current_index);
        int slow_velocity = slow_down(current_index);

        for (const auto& detection : msg->detections) {
            if (slow_velocity == 1 && detection.class_id == RED) {
                red_light_detected = true;
            }
            if (index_in_range == 1 && detection.class_id == RED) {
                if (!red_light_aeb_active) {
                    activateRedLightAEB();
                }
            }
            if (detection.class_id == 3) { // HUMAN detected
                human_detected = true;
                human_distance = detection.distance;
                if (!human_aeb_active && human_distance < 22.0) {
                    activateHumanAEB();
                }
            }
            if (detection.class_id == STOP_SIGN) { // STOP_SIGN detected
                activateStopAEB();  // AEB ON
            }
        }
    }

    bool is_on_intersection() {
        return current_road_option == TURN || current_road_option == STRAIGHT;
    }

    void run() {
        ros::Rate rate(10);  // 10 Hz
        while (ros::ok()) {
            float reference_velocity = 1.0;
            float should_increase_velocity = 0.0;
            if (velocity_increased == 1) {
                should_increase_velocity = 13.0;
                velocity_increased = 0;
            }

            if (aeb_active || collision_risk == AEB || human_aeb_active || red_light_aeb_active) {  
                reference_velocity = 9999.0;  // Placeholder value when AEB is active
                publishAEBStatus("AEB: ON");
            } else {
                publishAEBStatus("AEB: OFF");
                if (collision_risk == STOP || red_light_detected || human_detected) {
                    if (current_road_option == TURN) {
                        reference_velocity = 15.0 + should_increase_velocity;
                    } else if (current_road_option == STRAIGHT) {
                        reference_velocity = 15.0 + should_increase_velocity;
                    } else if (current_road_option == TWO_LANE) {
                        reference_velocity = 30.0 + should_increase_velocity;
                    } else if (current_road_option == SHOULDER) {
                        reference_velocity = 10.0 + should_increase_velocity;
                    }
                } else {
                    // Set reference velocity based on current road option
                    if (current_road_option == TURN) {
                        reference_velocity = 20.0 + should_increase_velocity;
                    } else if (current_road_option == STRAIGHT) {
                        reference_velocity = 30.0 + should_increase_velocity;
                    } else if (current_road_option == TWO_LANE) {
                        reference_velocity = 40.0 + should_increase_velocity;
                    } else if (current_road_option == SHOULDER) {
                        reference_velocity = 20.0 + should_increase_velocity;
                    } else {
                        reference_velocity = 20.0 + should_increase_velocity;
                    }
                }
            }

            // Publish the reference velocity
            std_msgs::Float32 velocity_msg;
            velocity_msg.data = reference_velocity;
            reference_velocity_pub.publish(velocity_msg);

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::Subscriber current_road_option_sub;
    ros::Subscriber collision_risk_sub;
    ros::Subscriber yolo_data_sub;
    ros::Subscriber velocity_increase_sub;
    ros::Subscriber current_index_sub;
    ros::Publisher reference_velocity_pub;
    ros::Publisher aeb_status_marker_pub;

    int collision_risk;
    int current_road_option;
    bool red_light_detected;
    bool human_detected;
    float human_distance;
    int velocity_increased;
    bool aeb_active;
    bool ignore_stop_sign;
    bool human_aeb_active;
    bool red_light_aeb_active;
    int current_index;
    ros::Timer aeb_timer;
    ros::Timer stop_sign_ignore_timer;
    ros::Timer human_aeb_timer;
    ros::Timer red_light_aeb_timer;

    void activateStopAEB() {
        if (!aeb_active) {
            aeb_active = true;
            aeb_timer = ros::NodeHandle().createTimer(ros::Duration(3.0), &VelocityPlanner::deactivateStopAEB, this, true); // 3초 후 AEB 비활성화
        }
    }

    void deactivateStopAEB(const ros::TimerEvent&) {
        aeb_active = false;
        // Turn off AEB, and wait for 10 sec
        ignore_stop_sign = true;
        stop_sign_ignore_timer = ros::NodeHandle().createTimer(ros::Duration(10.0), &VelocityPlanner::stopIgnoringStopSign, this, true);
    }

    void stopIgnoringStopSign(const ros::TimerEvent&) {
        ignore_stop_sign = false;
    }

    void activateHumanAEB() {
        if (!human_aeb_active) {
            human_aeb_active = true;
            human_aeb_timer = ros::NodeHandle().createTimer(ros::Duration(1.0), &VelocityPlanner::deactivateHumanAEB, this, true); // 1초 후 비활성화
        }
    }

    void deactivateHumanAEB(const ros::TimerEvent&) {
        human_aeb_active = false;
    }

    void activateRedLightAEB() {
        if (!red_light_aeb_active) {
            red_light_aeb_active = true;
            red_light_aeb_timer = ros::NodeHandle().createTimer(ros::Duration(1.0), &VelocityPlanner::deactivateRedLightAEB, this, true); // 1초 후 비활성화
        }
    }

    void deactivateRedLightAEB(const ros::TimerEvent&) {
        red_light_aeb_active = false;
    }

    void publishAEBStatus(const std::string& status) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link"; // 텍스트가 표시될 좌표계
        marker.header.stamp = ros::Time::now();
        marker.ns = "aeb_status";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 2.0; // RViz에서 텍스트가 약간 위쪽에 보이게끔 설정
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.z = 0.5; // 텍스트 크기
        marker.color.a = 1.0; // 불투명도
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        marker.text = status;

        aeb_status_marker_pub.publish(marker);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_planner");
    VelocityPlanner planner;
    planner.run();
    return 0;
}
