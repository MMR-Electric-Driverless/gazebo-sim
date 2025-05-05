#include "pure_pursuit/pure_pursuit.hpp"

PurePursuit::PurePursuit() : Node("pure_pursuit")
{
    load_parameters();

    waypoints_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
        waypoints_topic_, 10,
        std::bind(&PurePursuit::waypoint_callback, this, std::placeholders::_1));

    vehicle_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        vehicle_state_topic_, 10,
        std::bind(&PurePursuit::vehicle_state_callback, this, std::placeholders::_1));

    control_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(control_topic_, 10);
}

void PurePursuit::load_parameters()
{
    this->declare_parameter("target_speed", 5.0);
    this->declare_parameter("lookahead_distance", 2.0);
    this->declare_parameter("max_steering_angle", 0.523599); // 30 degrees in radians
    this->declare_parameter("wheelbase_length", 2.5);
    this->declare_parameter("waypoints_topic", "/waypoints");
    this->declare_parameter("vehicle_state_topic", "/vehicle_state");
    this->declare_parameter("control_topic", "/cmd_vel");
    this ->declare_parameter("p", 1.0);

    this->get_parameter("target_speed", targer_speed);
    this->get_parameter("lookahead_distance", lookahead_distance_);
    this->get_parameter("max_steering_angle", max_steering_angle_);
    this->get_parameter("wheelbase_length", wheelbase_length_);
    this->get_parameter("waypoints_topic", waypoints_topic_);
    this->get_parameter("vehicle_state_topic", vehicle_state_topic_);
    this->get_parameter("control_topic", control_topic_);
    this->get_parameter("p", this->p);
}

void PurePursuit::waypoint_callback(const visualization_msgs::msg::Marker::SharedPtr msg)
{
    // Store the waypoints in a vector
    for (auto point : msg->points)
    {
        waypoints_vector_.push_back(point);
    }
    RCLCPP_INFO(this->get_logger(), "Received waypoints.");
}

double get_angular_deviation(double a, double b) 
{
        double diff = std::fmod(b - a, 2.0 * M_PI);
        if (diff > M_PI)
            diff -= 2.0 * M_PI;
        else if (diff < -M_PI)
            diff += 2.0 * M_PI;
        return diff;
    
}

void PurePursuit::vehicle_state_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Extract the vehicle's current position and orientation
    double vehicle_x = msg->pose.pose.position.x;
    double vehicle_y = msg->pose.pose.position.y;

    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    double roll, pitch, vehicle_yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, vehicle_yaw);

    // Find the closest waypoint
    int closest_index = find_closest_waypoint(vehicle_x, vehicle_y);
    if (closest_index == -1)
    {
        RCLCPP_WARN(this->get_logger(), "No waypoints available.");
        return;
    }

    // Find the lookahead waypoint
    int lookahead_index = find_lookahead_waypoint(closest_index);
    if (lookahead_index == -1)
    {
        RCLCPP_WARN(this->get_logger(), "No valid lookahead waypoint found.");
        return;
    }

    // Extract the lookahead waypoint position
    double lookahead_x = waypoints_vector_[lookahead_index].x;
    double lookahead_y = waypoints_vector_[lookahead_index].y;

    // Calculate the relative position of the lookahead point in the vehicle's coordinate frame
    double dx = lookahead_x - vehicle_x;
    double dy = lookahead_y - vehicle_y;

    // Transform the lookahead point to the vehicle's coordinate frame
    double lookahead_x_vehicle = dx * std::cos(-vehicle_yaw) - dy * std::sin(-vehicle_yaw);
    double lookahead_y_vehicle = dx * std::sin(-vehicle_yaw) + dy * std::cos(-vehicle_yaw);

    // Calculate the steering angle using the pure pursuit formula
    double curvature = 2.0 * lookahead_y_vehicle / (lookahead_x_vehicle * lookahead_x_vehicle + lookahead_y_vehicle * lookahead_y_vehicle);
    double steering_angle = std::atan(curvature * wheelbase_length_);

    // Clamp the steering angle to the maximum steering angle
    steering_angle = std::max(-max_steering_angle_, std::min(max_steering_angle_, steering_angle));

    // Calculate the angular velocity command
    double command = p * steering_angle;

    // Publish the control command
    geometry_msgs::msg::Twist control_msg;
    control_msg.linear.x = targer_speed;
    control_msg.angular.z = command;
    control_pub_->publish(control_msg);
}

int PurePursuit::find_closest_waypoint(double x, double y)
{
    int closest_index = -1;
    double min_distance = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < this->waypoints_vector_.size(); ++i)
    {
        auto point = waypoints_vector_[i];
        double dx = point.x - x;
        double dy = point.y - y;
        double distance = std::sqrt(dx * dx + dy * dy);
        if (distance < min_distance)
        {
            min_distance = distance;
            closest_index = static_cast<int>(i);
        }
    }

    return closest_index;
}

int PurePursuit::find_lookahead_waypoint(int closest_index)
{        
    auto projection = waypoints_vector_[closest_index];

    for (size_t i = closest_index; i < this->waypoints_vector_.size(); ++i)
    {
        auto curr_point = waypoints_vector_[i];
        double dx = projection.x - curr_point.x;
        double dy = projection.y - curr_point.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        // Check if the distance is greater than or equal to the lookahead distance
        if (distance >= lookahead_distance_)
        {
            return static_cast<int>(i);
        }
    
    }

    return -1; // No valid lookahead waypoint found
}