#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <derived_object_msgs/ObjectArray.h>
#include <derived_object_msgs/Object.h>

class CarlaListener {
public:
    CarlaListener() : nh_("~"), ego_yaw_(0.0) {
        lead_vehicle_status_pub_ = nh_.advertise<geometry_msgs::Twist>("/carla/lead_vehicle_status", 10);

        // Subscribers
        gps_sub_ = nh_.subscribe("/carla/hero/GPS", 10, &CarlaListener::gpsCallback, this);
        imu_sub_ = nh_.subscribe("/carla/hero/IMU", 10, &CarlaListener::imuCallback, this);
        object_sub_ = nh_.subscribe("/carla/hero/Objects", 10, &CarlaListener::objectCallback, this);
    }

    void run() {
        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher lead_vehicle_status_pub_;
    ros::Subscriber gps_sub_, imu_sub_, object_sub_;

    double ego_yaw_;
    geometry_msgs::Point ego_position_;

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
        double roll, pitch;  // 임시 변수 선언
        tf::Quaternion quaternion(data->orientation.x, data->orientation.y, data->orientation.z, data->orientation.w);
        tf::Matrix3x3(quaternion).getRPY(roll, pitch, ego_yaw_);  // 임시 변수 사용
    }


    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& data) {
        double lat0 = 0.0, lon0 = 0.0, alt0 = 0.0;
        llaToENU(data->latitude, data->longitude, data->altitude, lat0, lon0, alt0, ego_position_.x, ego_position_.y, ego_position_.z);
    }

    double angleMod(double angle) {
        return fmod(angle + M_PI, 2 * M_PI) - M_PI;
    }

    void objectCallback(const derived_object_msgs::ObjectArray::ConstPtr& data) {
        std::vector<const derived_object_msgs::Object*> closest_objects(2, nullptr);
        std::vector<double> min_distances(2, std::numeric_limits<double>::infinity());
        std::vector<int> candidate_ids;

        if (ego_yaw_ != 0) {
            for (const auto& obj : data->objects) {
                double distance = calculateDistance(ego_position_, obj.pose.position);

                // 내 차량과 같은 위치인지 확인하여 제외
                if (distance < 1.0)
                    continue;

                // classification이 6인지 확인
                if (obj.classification == 6) {
                    double obj_yaw = tf::getYaw(obj.pose.orientation);

                    // 헤딩이 같은 방향으로 향하는 객체 필터링 (예: ±10도 내외)
                    bool valid_angle = false;
                    if (0.785398 <= ego_yaw_ && ego_yaw_ < 2.35619) {  // 45도 ~ 135도
                        valid_angle = std::abs(angleMod(obj_yaw - ego_yaw_)) < 0.174;
                    } else if ((2.35619 <= ego_yaw_ && ego_yaw_ <= 3.14159) || (-3.14159 <= ego_yaw_ && ego_yaw_ < -2.35619)) {  // 135도 ~ 180도 or -180도 ~ -135도
                        valid_angle = std::abs(angleMod(obj_yaw - ego_yaw_)) < 0.174;
                    } else if (-2.35619 <= ego_yaw_ && ego_yaw_ < -0.785398) {  // -135도 ~ -45도
                        valid_angle = std::abs(angleMod(obj_yaw - ego_yaw_)) < 0.174;
                    } else {  // 그 외의 각도
                        valid_angle = std::abs(angleMod(obj_yaw - ego_yaw_)) < 0.174;
                    }

                    if (valid_angle) {
                        if (distance <= 30.0) {
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
        }

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
