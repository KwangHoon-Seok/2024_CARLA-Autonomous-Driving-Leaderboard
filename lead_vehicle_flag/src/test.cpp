#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "marker_example_node");
    ros::NodeHandle nh;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ros::Rate r(10);  // 10 Hz

    while (ros::ok()) {
        visualization_msgs::Marker marker;

        // Set the frame ID to "hero"
        marker.header.frame_id = "hero";
        marker.header.stamp = ros::Time::now();

        marker.ns = "example_namespace";
        marker.id = 0;

        marker.type = visualization_msgs::Marker::CUBE;  // Type of the marker
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker relative to the "hero" frame
        marker.pose.position.x = 1.0;
        marker.pose.position.y = 2.0;
        marker.pose.position.z = 0.5;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker (size of the box)
        marker.scale.x = 1.0;  // Width
        marker.scale.y = 1.0;  // Length
        marker.scale.z = 1.0;  // Height

        // Set the color of the marker
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;  // Alpha (opacity)

        // Publish the marker
        marker_pub.publish(marker);

        r.sleep();
    }
}
