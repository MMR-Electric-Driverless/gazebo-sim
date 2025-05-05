# pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>


class PurePursuit : public rclcpp::Node
{
public:
    PurePursuit();
    void load_parameters();
    void waypoint_callback(const visualization_msgs::msg::Marker::SharedPtr msg);
    void vehicle_state_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

private:
    double targer_speed;
    double lookahead_distance_;
    double max_steering_angle_;
    double wheelbase_length_;
    double p;

    std::string waypoints_topic_;
    std::string vehicle_state_topic_;
    std::string control_topic_;

    std::vector<geometry_msgs::msg::Point> waypoints_vector_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_state_sub_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr waypoints_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_pub_;

    int find_closest_waypoint(double x, double y);
    int find_lookahead_waypoint(int closest_index);
};
