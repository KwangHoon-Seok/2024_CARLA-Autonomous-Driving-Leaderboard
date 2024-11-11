// Extented Kalman Filter, state = [x, y, yaw], input = []
#include <iostream>
#include <vector>
#include <cmath>
#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <carla_msgs/CarlaSpeedometer.h>  // For speedometer data
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <tf/tf.h> // For converting quaternion to yaw

using namespace std;
using namespace Eigen;

const double EARTH_RADIUS_EQUA = 6378137.0;  // Earth's radius in meters

double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

// void lla_to_enu(double lat, double lon, double alt, double lat0, double lon0, double alt0, double& e, double& n, double& u) {
//     double scale = cos(deg2rad(lat0));

//     double mx = scale * deg2rad(lon) * EARTH_RADIUS_EQUA;
//     double my = scale * EARTH_RADIUS_EQUA * log(tan(M_PI / 4.0 + deg2rad(lat) / 2.0));

//     double mx_ref = scale * deg2rad(lon0) * EARTH_RADIUS_EQUA;
//     double my_ref = scale * EARTH_RADIUS_EQUA * log(tan(M_PI / 4.0 + deg2rad(lat0) / 2.0));

//     e = mx - mx_ref;
//     n = my - my_ref;
//     u = alt - alt0;
// }

void lla_to_enu(double lat, double lon, double alt, double lat0, double lon0, double alt0, double& e, double& n, double& u) {
    double scale = cos(lat0 * M_PI / 180.0);

    double mx_ref = lon0 * M_PI / 180.0 * EARTH_RADIUS_EQUA;
    double my_ref = EARTH_RADIUS_EQUA * log(tan((90.0 + lat0) * M_PI / 360.0));
    double mx = lon * M_PI / 180.0 * EARTH_RADIUS_EQUA;
    double my = EARTH_RADIUS_EQUA * log(tan((90.0 + lat) * M_PI / 360.0));

    e = scale * (mx - mx_ref);
    n = scale * (my - my_ref);
    u = alt - alt0;
}



class KalmanFilter {
public:
    KalmanFilter(VectorXd init_state, double init_time, double pos_var, double yaw_var, double meas_xy_var, double meas_yaw_var, double alpha = 1.0) {
        state = init_state;
        prev_time = init_time;
        P = MatrixXd::Zero(3, 3);

        Q = MatrixXd::Zero(3, 3);
        Q(0, 0) = Q(1, 1) = pos_var;
        Q(2, 2) = yaw_var;

        R = MatrixXd::Zero(3, 3);
        R(0, 0) = R(1, 1) = meas_xy_var;
        R(2, 2) = meas_yaw_var;

        speedometer_speed = 0.0; // Initialize speedometer speed
    }

    void set_initial_covar(const MatrixXd& P_init) {
        P = P_init;
    }

    void prediction(Vector2d inp, double time) {
        double dt = time - prev_time;
        double v = inp(0);  // Speed from speedometer
        double yaw_rate = inp(1);  // Yaw rate from IMU

        // Update position based on the updated yaw
        state(0) += v * dt * cos(state(2));  // x position
        state(1) += v * dt * sin(state(2));  // y position
        state(2) += yaw_rate * dt; // yaw

        // Make yaw in boundary (-pi, pi)
        while (state(2) > M_PI) state(2) -= 2 * M_PI;
        while (state(2) < -M_PI) state(2) += 2 * M_PI;

        // Jacobian matrix F calculation
        MatrixXd F(3, 3);
        F << 1, 0, -v * dt * sin(state(2)),
            0, 1, v * dt * cos(state(2)),
            0, 0, 1;

        // Predict the error covariance matrix
        P = F * P * F.transpose() + Q;

        prev_time = time;
    }

    void measurement_update(VectorXd measurement, double time) {
        MatrixXd H(3, 3);
        H << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;

        VectorXd V = measurement - H * state; // 3*1
        MatrixXd S = H * P * H.transpose() + R; // 3*3
        MatrixXd K = P * H.transpose() * S.inverse(); // 3*3

        state = state + K * V; // 3*1
        P = P - K * H * P; // 3*3
    }

    VectorXd get_pos() {
        return state;
    }

    double get_covar_mean() const {
        return P.mean();  // Calculate the mean of the covariance matrix
    }

    void set_speedometer_speed(double speed, double current_time) {
        speedometer_speed = speed;
    }

    double get_speedometer_speed() const {
        return speedometer_speed;
    }

    void update_params(double pos_var, double yaw_var, double meas_xy_var, double meas_yaw_var) {
        Q(0, 0) = pos_var;
        Q(1, 1) = pos_var;
        Q(2, 2) = yaw_var;
        R(0, 0) = meas_xy_var;
        R(1, 1) = meas_xy_var;
        R(2, 2) = meas_yaw_var;
    }

private:
    VectorXd state;
    double prev_time;
    MatrixXd P;
    MatrixXd Q;
    MatrixXd R;
    double speedometer_speed;  // Variable to store speed from speedometer
    double yaw_ema;  // Exponential Moving Average for yaw
    double alpha;    // Smoothing factor
};

void print_position(const string& label, const VectorXd& state, double covar_mean) {
    cout << label << " Position:" << endl;
    cout << "  x: " << state(0) << endl;
    cout << "  y: " << state(1) << endl;
    cout << "  yaw: " << state(2) << endl;
    cout << "  Covariance Mean: " << covar_mean << endl;  // Print the covariance mean
}

// Global variables to hold state
KalmanFilter* kf;
double lat0 = 0, lon0 = 0, alt0 = 0;
sensor_msgs::NavSatFix last_gnss_msg; // Store the last GNSS message

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    ROS_INFO("GPS callback!!");
    last_gnss_msg = *msg;  // Save the GNSS data for later use in IMU callback
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    ROS_INFO("IMU callback!!");
    
    // Convert quaternion to yaw
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    double roll, pitch, yaw_measured;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw_measured);

    // Create the input vector for the Kalman filter
    Vector2d inp;
    inp << kf->get_speedometer_speed(), msg->angular_velocity.z;  // Speed and yaw rate

    // Run prediction with the input
    kf->prediction(inp, ros::Time::now().toSec());

    // Use the last GNSS message to get the ENU position
    double e, n, u;
    lla_to_enu(last_gnss_msg.latitude, last_gnss_msg.longitude, last_gnss_msg.altitude, lat0, lon0, alt0, e, n, u);

    // Measurement update with yaw and GNSS-provided position
    VectorXd measurement(3);

    // Make yaw in boundary (-pi, pi)
    while (yaw_measured > M_PI) yaw_measured -= 2 * M_PI;
    while (yaw_measured < -M_PI) yaw_measured += 2 * M_PI;
    measurement << e, n, yaw_measured;  // Use GNSS position and measured yaw

    kf->measurement_update(measurement, ros::Time::now().toSec());
}

void speedometer_callback(const carla_msgs::CarlaSpeedometer::ConstPtr& msg) {
    ROS_INFO("Speedometer callback!!");
    double speed = msg->speed;  // Assume CarlaSpeedometer gives speed in m/s
    double current_time = ros::Time::now().toSec();
    kf->set_speedometer_speed(speed, current_time);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle nh;

    // Initialize Kalman filter
    VectorXd init_state(3);
    init_state << 0, 0, 0;
    double init_time = ros::Time::now().toSec();
    double pos_var = 0.0000001;
    double yaw_var = 1.0;
    double meas_xy_var = 2.0;
    double meas_yaw_var = 1.0;

    MatrixXd P_init(3, 3);
    double init_var = 1000000;
    P_init << init_var, 0, 0,
              0, init_var, 0,
              0, 0, init_var;

    kf = new KalmanFilter(init_state, init_time, pos_var, yaw_var, meas_xy_var, meas_yaw_var);
    kf->set_initial_covar(P_init);

    ros::Subscriber gps_sub = nh.subscribe("/carla/hero/GPS", 1000, gps_callback);
    ros::Subscriber imu_sub = nh.subscribe("/carla/hero/IMU", 1000, imu_callback);
    ros::Subscriber speedometer_sub = nh.subscribe("/carla/hero/Speed", 1000, speedometer_callback);  // Subscribing to speedometer data
    ros::Publisher enu_pub = nh.advertise<geometry_msgs::Point>("enu_coordinates", 10);
    ros::Rate rate(20);  // 20 Hz

    while (ros::ok()) {
        // Update Kalman filter parameters if they change
        nh.getParam("localization_node/pos_var", pos_var);
        nh.getParam("localization_node/yaw_var", yaw_var);
        nh.getParam("localization_node/meas_xy_var", meas_xy_var);
        nh.getParam("localization_node/meas_yaw_var", meas_yaw_var);
        kf->update_params(pos_var, yaw_var, meas_xy_var, meas_yaw_var);

        ros::spinOnce();
        VectorXd pos = kf->get_pos();
        double covar_mean = kf->get_covar_mean();
        print_position("Kalman Filter", pos, covar_mean);
        cout << "pos_var : " << pos_var << endl;
        cout << "yaw_var : " << yaw_var << endl;
        cout << "meas_xy_var : " << meas_xy_var << endl;
        cout << "meas_yaw_var : " << meas_yaw_var << endl;

        geometry_msgs::Point enu_msg;
        enu_msg.x = pos(0);
        enu_msg.y = pos(1);
        enu_msg.z = pos(2);
        enu_pub.publish(enu_msg);

        rate.sleep();
    }

    delete kf;
    return 0;
}


