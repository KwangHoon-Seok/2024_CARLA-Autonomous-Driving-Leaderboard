// rosrun dwa_ros_node dwa_node
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <carla_msgs/CarlaSpeedometer.h>
#include <tf/tf.h>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <nav_msgs/Path.h>
#include <chrono>
#include <fstream>
#include <thread>
#include <atomic>
#include <iostream>
#include <your_package/DetectionInfoArray.h>
#include <your_package/DetectionInfo.h>


using Eigen::MatrixXd;

class Config {
public:
    double max_speed = 15.0 / 3.6;           // 최대 속도 (m/s)
    double min_speed = 0.0;                  // 최소 속도 (m/s)
    double max_yaw_rate = 70.0;              // 최대 yawrate (deg/s)
    double max_steer_angle = 70.0;           // 최대 조향 각도 (deg)
    double max_accel = 10.0 / 3.6;           // 최대 가속도 (m/s^2)
    double max_delta_yaw_rate = 43.0;        // 최대 yawrate 변화율 (deg/s^2)
    double v_resolution = 1 / 3.6;           // 속도 해상도 (m/s)
    double yaw_rate_resolution = 0.2;        // yawrate 해상도 (deg/s)
    double dt = 0.1;                         // 시간 간격 (s)
    double predict_time = 0.5;               // 예측 시간 (s)

    double to_goal_cost_gain = 1.0;          // 목표점까지의 거리 비용 가중치
    double speed_cost_gain = 0.3;            // 속도 비용 가중치
    double obstacle_cost_gain = 2.0;           // 장애물 회피 비용 가중치

    double vehicle_stuck_flag_cons = 0.001;  // 차량 정지 여부 확인 임계값
    double vehicle_width = 1.85;             // 차량 너비 (m)
    double vehicle_length = 4.72;            // 차량 길이 (m)
    double wheelbase = 2.875;                // 휠베이스 길이 (m)
    double lf = 1.3976;                      // 전방 축 거리 (m)
    double lr = 1.4774;                      // 후방 축 거리 (m)
};

class DWA {
public:
    DWA(const std::vector<double>& goal, const MatrixXd& obstacle) : _goal(goal), _obj_dis(obstacle) {
        _config = Config();
    }
    // 최대 속도 반환
    double getMaxSpeed() const {
        return _config.max_speed;
    }
    // dynamic window 계산
    std::vector<double> get_dynamic_window(const std::vector<double>& state) {
        return _calc_dynamic_window(state);
    }
    // DWA 제어 계산
    std::pair<std::vector<double>, MatrixXd> dwa_control(const std::vector<double>& state) {
        std::vector<double> dw = _calc_dynamic_window(state);
        auto result = _calc_control_and_trajectory(dw, state);
        return result;
    }

private:
    Config _config;
    std::vector<double> _goal;
    MatrixXd _obj_dis;
    // 현재 속도에 따른 최대 yawrate 계산
    double _calc_curr_max_yaw_rate(double current_speed) {
        if (current_speed == 0) {
            return 0; 
        }
        return current_speed * std::tan(_config.max_steer_angle * M_PI / 180) / _config.wheelbase;
    }
    // Dynamic Window 계산
    std::vector<double> _calc_dynamic_window(const std::vector<double>& state) {
        std::vector<double> Vs = {_config.min_speed, _config.max_speed, -_config.max_yaw_rate, _config.max_yaw_rate};
        double curr_max_yaw_rate = _calc_curr_max_yaw_rate(state[3]);

        double current_yaw_rate = state[4];
        double next_yaw_rate_right = current_yaw_rate + _config.max_delta_yaw_rate * _config.dt;
        double next_yaw_rate_left = current_yaw_rate - _config.max_delta_yaw_rate * _config.dt;

        if (current_yaw_rate > 0) { 
            next_yaw_rate_right = std::min(next_yaw_rate_right, curr_max_yaw_rate); 
        } else if (current_yaw_rate < 0) { 
            next_yaw_rate_left = std::max(next_yaw_rate_left, -curr_max_yaw_rate); 
        }
        // 실제 Dynamic Window  범위
        std::vector<double> Vd = {state[3] - _config.max_accel * _config.dt, state[3] + _config.max_accel * _config.dt,
                                  next_yaw_rate_left, next_yaw_rate_right};

        std::vector<double> dw = {std::max(Vs[0], Vd[0]), std::min(Vs[1], Vd[1]), std::max(Vs[2], Vd[2]), std::min(Vs[3], Vd[3])};

        // ROS_INFO("Dynamic Window - Speed: [%f, %f], Yaw Rate: [%f, %f]", dw[0], dw[1], dw[2], dw[3]);

        return dw;
    }
    // 제어입력을 받아 궤적 계산
    std::pair<std::vector<double>, MatrixXd> _calc_control_and_trajectory(const std::vector<double>& dw, const std::vector<double>& state) {
        auto start = std::chrono::high_resolution_clock::now();

        std::vector<double> x_init = state;
        double min_cost = std::numeric_limits<double>::infinity();
        std::vector<double> best_u = {0.0, 0.0};
        MatrixXd best_trajectory(1, x_init.size());
        best_trajectory.row(0) = Eigen::Map<const MatrixXd>(x_init.data(), 1, x_init.size());
        std::vector<double> last_best_u = best_u;

        for (double v = dw[0]; v <= dw[1]; v += _config.v_resolution) {
            for (double y = dw[2]; y <= dw[3]; y += _config.yaw_rate_resolution) {
                MatrixXd trajectory = _predict_trajectory(x_init, v, y);
                double to_goal_cost = _config.to_goal_cost_gain * _calc_to_goal_cost(trajectory);
                double speed_cost = _config.speed_cost_gain * (_config.max_speed - trajectory(trajectory.rows() - 1, 3));
                double ob_cost = _config.obstacle_cost_gain * _calc_obstacle_cost(trajectory);
                double final_cost = to_goal_cost + speed_cost + ob_cost;

                if (min_cost > final_cost) {
                    min_cost = final_cost;
                    best_u = {v, y};
                    best_trajectory = trajectory;
                }
            }
        }
        if (last_best_u != best_u) {
        //    ROS_INFO("Evaluated Path - Speed: %f, Yaw Rate: %f", best_u[0], best_u[1]);
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        // ROS_INFO("Execution time of _calc_control_and_trajectory: %f seconds", elapsed.count());

        return {best_u, best_trajectory};
    }
    // 궤적 예측
    MatrixXd _predict_trajectory(const std::vector<double>& x_init, double v, double y) {
        MatrixXd x = Eigen::Map<const MatrixXd>(x_init.data(), 1, x_init.size());
        MatrixXd trajectory = x;
        double time = 0;

        while (time <= _config.predict_time) {
            x = motion(x, {v, y}, _config.dt);
            trajectory.conservativeResize(trajectory.rows() + 1, trajectory.cols());
            trajectory.row(trajectory.rows() - 1) = x;
            time += _config.dt;
        }
        return trajectory;
    }
    // 움직임 예측
    MatrixXd motion(const MatrixXd& x, const std::vector<double>& u, double dt) {
        MatrixXd new_x = x;
        double velocity = u[0];
        double yaw_rate = u[1];

        double yaw = x(0, 2) * M_PI / 180.0;

        double delta = atan2(yaw_rate * _config.wheelbase, velocity) * 180.0 / M_PI;
        double beta = atan((_config.lr / _config.wheelbase) * tan(delta * M_PI / 180.0));

        new_x(0, 0) += velocity * cos(yaw + beta) * dt;
        new_x(0, 1) += velocity * sin(yaw + beta) * dt;
        new_x(0, 2) += yaw_rate * dt * 180.0 / M_PI;
        new_x(0, 3) = velocity;
        new_x(0, 4) = yaw_rate;

        return new_x;
    }
    // obstacle cost 계산
    double _calc_obstacle_cost(const MatrixXd& trajectory) {
        if (_obj_dis.size() == 0) {
            return 0;
        }

        double min_distance = std::numeric_limits<double>::infinity();

        Eigen::Vector2d obstacle_start(_obj_dis(0, 0), _obj_dis(0, 1));
        Eigen::Vector2d obstacle_end(_obj_dis(_obj_dis.rows() - 1, 0), _obj_dis(_obj_dis.rows() - 1, 1));

        for (int i = 0; i < trajectory.rows(); ++i) {
            Eigen::Vector2d point(trajectory(i, 0), trajectory(i, 1));

            double cross_product = (obstacle_end[0] - obstacle_start[0]) * (point[1] - obstacle_start[1]) -
                                (obstacle_end[1] - obstacle_start[1]) * (point[0] - obstacle_start[0]);

            for (int j = 0; j < _obj_dis.rows(); ++j) {
                double distance = std::sqrt(std::pow(trajectory(i, 0) - _obj_dis(j, 0), 2) +
                                            std::pow(trajectory(i, 1) - _obj_dis(j, 1), 2));
                if (distance < min_distance) {
                    min_distance = distance;
                }
            }
        }

        return 1.0 / min_distance;
    }
    // goal cost 계산
    double _calc_to_goal_cost(const MatrixXd& trajectory) {
        double dx = _goal[0] - trajectory(trajectory.rows() - 1, 0);
        double dy = _goal[1] - trajectory(trajectory.rows() - 1, 1);
        return std::sqrt(dx * dx + dy * dy);
    }
    // 차량의 네 모서리 사용
    MatrixXd _get_vehicle_corners(const Eigen::VectorXd& x, const Eigen::VectorXd& y, const Eigen::VectorXd& yaw) {
        double width = _config.vehicle_width;
        double length = _config.vehicle_length;
        MatrixXd corners(x.size(), 8);

        for (int i = 0; i < x.size(); ++i) {
            double cos_yaw = std::cos(yaw[i]);
            double sin_yaw = std::sin(yaw[i]);
            double half_width = width / 2.0;
            double half_length = length / 2.0;

            corners(i, 0) = x[i] + half_length * cos_yaw + half_width * sin_yaw;
            corners(i, 1) = y[i] + half_length * sin_yaw - half_width * cos_yaw;
            corners(i, 2) = x[i] + half_length * cos_yaw - half_width * sin_yaw;
            corners(i, 3) = y[i] + half_length * sin_yaw + half_width * cos_yaw;
            corners(i, 4) = x[i] - half_length * cos_yaw + half_width * sin_yaw;
            corners(i, 5) = y[i] - half_length * sin_yaw - half_width * cos_yaw;
            corners(i, 6) = x[i] - half_length * cos_yaw - half_width * sin_yaw;
            corners(i, 7) = y[i] - half_length * sin_yaw + half_width * cos_yaw;
        }
        return corners;
    }
    // 장애물과 네 모서리의 위치 비교 -> obstacle cost = "inf"
    bool _is_point_inside_rectangle(const Eigen::RowVectorXd& rectangle, double p_x, double p_y) {
        Eigen::Vector2d p(p_x, p_y);

        Eigen::Vector2d r0 = rectangle.segment<2>(0);
        Eigen::Vector2d r1 = rectangle.segment<2>(2);
        Eigen::Vector2d r2 = rectangle.segment<2>(4);
        Eigen::Vector2d r3 = rectangle.segment<2>(6);

        auto cross_product = [](const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) {
            return v1[0] * v2[1] - v1[1] * v2[0];
        };

        Eigen::Vector2d r0p = p - r0;
        Eigen::Vector2d r1p = p - r1;
        Eigen::Vector2d r2p = p - r2;
        Eigen::Vector2d r3p = p - r3;

        bool is_inside = (cross_product(r1 - r0, r0p) >= 0 &&
                        cross_product(r2 - r1, r1p) >= 0 &&
                        cross_product(r3 - r2, r2p) >= 0 &&
                        cross_product(r0 - r3, r3p) >= 0);

        return is_inside;
    }
};

class DWANode { 
public:
    DWANode() {
        traj_pub = nh.advertise<geometry_msgs::PoseArray>("/dwa_trajectory", 10);
        control_pub = nh.advertise<std_msgs::Float32MultiArray>("/dwa_control_input", 10);
        local_planning_sub = nh.subscribe("/local_planning", 10, &DWANode::driveModeCallback, this);
        local_planning_pub = nh.advertise<std_msgs::Bool>("/local_planning", 10);
        enu_sub = nh.subscribe("/enu_coordinates", 10, &DWANode::enu_callback, this);
        imu_sub = nh.subscribe("/carla/hero/IMU", 10, &DWANode::imu_callback, this);
        speed_sub = nh.subscribe("/carla/hero/Speed", 10, &DWANode::speed_callback, this);
        class_sub = nh.subscribe("detection_with_distance", 10, &DWANode::obstacle_callback, this); // [class id = 1, width, heidht, center_x, center_y , 거리]
        path_pub = nh.advertise<nav_msgs::Path>("/vehicle_path", 10);
        vehicle_path.header.frame_id = "map";
        current_speed = 0.0;

        user_input_thread = std::thread(&DWANode::handleUserInput, this);

        // vehicle_stuck_flag_cons = 0.001;

        local_planning = false;
        dwa_initialized = false;
        obstacle_distance = std::numeric_limits<float>::infinity();

        csv_file.open("ego_vehicle_positions.csv");
        csv_file << "Time, X, Y, Heading, Speed\n";  

        ROS_INFO("DWANode initialized");
    }

    ~DWANode() {
        if (csv_file.is_open()) {
            csv_file.close();
        }

        if (user_input_thread.joinable()) {
            user_input_thread.join();
        }

    }

    void spin() {
        ros::Rate rate(20);

        while (ros::ok()) {
            if (local_planning == true && !dwa_initialized) {
                initializeDWA();
                dwa_initialized = true;
            }

            if (local_planning == true && dwa_initialized) {
                double max_speed = dwa->getMaxSpeed();
                if (current_speed > max_speed) {
                    current_speed = max_speed;
                } else if ( current_speed == 0) {
                    current_speed = 0.001;
                }

                auto result = dwa->dwa_control(init_state);
                auto u = result.first;
                auto trajectory = result.second;

                publish_trajectory(trajectory);

                std_msgs::Float32MultiArray control_msg;
                control_msg.data.resize(u.size());
                for (size_t i = 0; i < u.size(); ++i) {
                    control_msg.data[i] = static_cast<float>(u[i]);
                }
                control_pub.publish(control_msg);

                init_state[0] = x;
                init_state[1] = y;
                init_state[2] = heading;
                init_state[3] = current_speed;
                init_state[4] = u[1];

                if (std::hypot(init_state[0] - goal[0], init_state[1] - goal[1]) < 1.0) {
                    ROS_INFO("Finish");

                    std_msgs::Bool local_planning_msg;
                    local_planning_msg.data = false;
                    local_planning_pub.publish(local_planning_msg);
                    local_planning = false;

                }
            }
            log_position_to_csv();

            ros::spinOnce();
            rate.sleep();
        }
        // ROS_INFO("DWA End");
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber enu_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber speed_sub;
    ros::Subscriber local_planning_sub;
    ros::Subscriber class_sub;
    ros::Publisher traj_pub;
    ros::Publisher control_pub;
    ros::Publisher local_planning_pub;
    ros::Publisher path_pub;
    your_package::DetectionInfoArray detections_;
    
    // ros::Subscriber user_input_sub; // --------------------------------------------------------------------------


    nav_msgs::Path vehicle_path;

    int class_id;
    double x, y;
    double heading;
    double current_speed;
    float obstacle_distance;
    // bool local_planning;
    // bool dwa_initialized; ------------------------------------------------------
    std::atomic<bool> local_planning;  // atomic for thread safety ---------------
    std::atomic<bool> dwa_initialized; // atomic for thread safety

    bool received_enu = false;
    bool received_imu = false;

    std::vector<double> init_state;
    std::vector<double> goal;
    std::unique_ptr<DWA> dwa;

    std::ofstream csv_file;
    std::thread user_input_thread;


    void initializeDWA() {
        init_state = {x, y, heading, current_speed, 0.0};

        double offset_x = 20.0;
        double offset_y = 0.5;
        double yaw = init_state[2];
        double goal_yaw;

        if (yaw >= -45.0 && yaw < 45.0) {
            goal_yaw = 0.0;
        } else if (yaw >= 45.0 && yaw < 135.0) {
            goal_yaw = M_PI / 2;
        } else if ((yaw >= 135.0 && yaw <= 180.0) || (yaw >= -180.0 && yaw < -135.0)) {
            goal_yaw = M_PI;
        } else if (yaw >= -135.0 && yaw < -45.0) {
            goal_yaw = -M_PI / 2;
        }

        double goal_x = init_state[0] + offset_x * cos(goal_yaw) - offset_y * sin(goal_yaw);
        double goal_y = init_state[1] + offset_x * sin(goal_yaw) + offset_y * cos(goal_yaw);
        ROS_INFO("goal point = %f, %f", goal_x, goal_y);
        goal = {goal_x, goal_y};

        double distance_forward = 6;  // 문 끝까지의 거리
        // double distance_forward = obstacle_distance;  // 문 끝까지의 거리
        double distance_right = 2.5;

        double x_start = init_state[0] + distance_forward * cos(goal_yaw);
        double y_start = init_state[1] + distance_forward * sin(goal_yaw);

        double x_end = x_start + distance_right * cos(goal_yaw - M_PI / 2);
        double y_end = y_start + distance_right * sin(goal_yaw - M_PI / 2);
        int num_points = 11;

        MatrixXd obstacle(num_points, 2);
        for (int i = 0; i < num_points; ++i) {
            double t = static_cast<double>(i) / (num_points - 1);
            obstacle(i, 0) = (1 - t) * x_start + t * x_end;
            obstacle(i, 1) = (1 - t) * y_start + t * y_end;
        }

        dwa = std::make_unique<DWA>(goal, obstacle);
        dwa_initialized = true;
    }

    void enu_callback(const geometry_msgs::Point::ConstPtr& msg) {
        // ROS_INFO("ENU callback triggered");

        x = msg->x;
        y = msg->y;
        received_enu = true;
        // ROS_INFO("ENU data processed, received_enu set to true");
        // ROS_INFO("ENU: [%f, %f]", x, y);
    }

    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
        tf::Quaternion quaternion;
        tf::quaternionMsgToTF(msg->orientation, quaternion);

        double roll, pitch, yaw;
        tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        heading = yaw * 180.0 / M_PI;
        received_imu = true;
    }

    void speed_callback(const carla_msgs::CarlaSpeedometer::ConstPtr& msg) {
        current_speed = msg->speed;
    }

    void driveModeCallback(const std_msgs::Bool::ConstPtr& msg) {
        local_planning = msg->data;
        // ROS_INFO("Drive mode updated: %d", local_planning); --------------------------------------------------------
        ROS_INFO("Drive mode updated: %d", static_cast<bool>(local_planning)); // Explicitly cast to bool     
    }

    void obstacle_callback(const your_package::DetectionInfoArray::ConstPtr& msg) {   
        detections_ = *msg;

        obstacle_distance = std::numeric_limits<float>::infinity();

        for (const auto& detection : detections_.detections) {
            class_id = detection.class_id;
        
            if (class_id == 1){
                float distance = detection.distance;
                obstacle_distance = distance;
                // ROS_INFO("distance to obstacle : {%f}", obstacle_distance);
                            
                if (!local_planning && !dwa_initialized) {
                    local_planning = true;
                    ROS_INFO("Class ID 1 detected. Local planning flag set to true.");

                    // local_planning_flag를 true로 설정하면 바로 DWA 초기화를 수행할 수 있습니다.
                    initializeDWA();

                    // local_planning을 true로 설정한 사실을 알리기 위해 메시지를 발행합니다.
                    std_msgs::Bool local_planning_msg;
                    local_planning_msg.data = true;
                    local_planning_pub.publish(local_planning_msg);
                }
            }
        }
    }



    void handleUserInput() {
        while (ros::ok()) {
            int input;
            std::cin >> input;

            if (input == 0) {
                // ROS_INFO("User input 0 received. Entering wait mode.");
                local_planning = false;  // 대기 상태로 전환
                dwa_initialized = false; // 초기화 플래그 리셋
            }
        }
    }

    
    void publish_trajectory(const MatrixXd& trajectory) {
        geometry_msgs::PoseArray pose_array;
        pose_array.header.stamp = ros::Time::now();
        pose_array.header.frame_id = "map";

        for (int i = 0; i < trajectory.rows(); ++i) {
            geometry_msgs::Pose pose;
            pose.position.x = trajectory(i, 0);
            pose.position.y = trajectory(i, 1);
            pose.position.z = 0;
            pose.orientation.w = 1.0;
            pose_array.poses.push_back(pose);

            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose = pose;
            vehicle_path.poses.push_back(pose_stamped);
        }

        traj_pub.publish(pose_array);
        path_pub.publish(vehicle_path);
    }

    void log_position_to_csv() {
        if (csv_file.is_open()) {
            ros::Time now = ros::Time::now();
            csv_file << now << " , " << x << " , " << y << " , " << heading << " , " << current_speed << "\n";
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dwa_node");

    DWANode dwa_node;
    dwa_node.spin();
    // ROS_INFO("Spin method exited");

    return 0;
}
