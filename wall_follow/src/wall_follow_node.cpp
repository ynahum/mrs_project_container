#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <string>
#include <typeinfo>
#include <cmath>
#define _USE_MATH_DEFINES

using namespace std;
using namespace sensor_msgs::msg;
using namespace nav_msgs::msg;
using namespace ackermann_msgs::msg;

double deg2rad(double deg) 
{
    return deg * M_PI / 180.0;
}

double rad2deg(double rad) 
{
    return rad * 180.0 / M_PI;
}

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        drive_pub_ = this->create_publisher<AckermannDriveStamped>(drive_topic, 10);
        lidar_sub_ = this->create_subscription<LaserScan>(
            lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));
        theta_rad_ = deg2rad(theta_degrees_);
        prev_t_ = get_clock()->now().seconds();
    }

private:
    // PID CONTROL PARAMS
    double kp_ = 1.2;
    double kd_ = 0.1;
    double ki_ = 0;//0.0000015;

    bool first_scan_read_ = true;
    double angle_min_ = 0.0;
    double angle_range_ = 0.0;
    uint32_t ranges_size_ = 0;
    uint32_t left_index_ = 0;
    double theta_rad_ = 0;

    double prev_t_ = 0.0;
    double prev_error_ = 0.0;
    double error_integral_ = 0.0;

    bool is_dead_end_ = false;

    const uint32_t theta_degrees_ = 35;
    const double DIST = 0.9;
    const double L = 0.5; // Lookahead distance in meters
    const double theta_front_left_ = 45;
    const double theta_front_right_ = -45;

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    rclcpp::Publisher<AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<LaserScan>::SharedPtr lidar_sub_;

    double get_range(const float* range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */
        uint32_t index = (uint32_t)round(ranges_size_*(angle-angle_min_)/angle_range_);
        assert(index <= ranges_size_);
        return range_data[index];
    }

    double get_error(const float* range_data, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        double b = get_range(range_data, M_PI_2);
        double a = get_range(range_data, M_PI_2 - theta_rad_);

        double n = a * cos(theta_rad_) - b;
        double d = a * sin(theta_rad_);
        double alpha = atan(n / d);

        double d_t = b * cos(alpha);

        double d_tp1 = d_t + L * sin(alpha);
        return d_tp1 - dist;
    }

    void pid_control(double error)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        auto drive_msg = std::make_shared<AckermannDriveStamped>();
        double t = get_clock()->now().seconds();
        double delta_t = t- prev_t_;
        error_integral_ += delta_t * error;
        double error_derivative = (error - prev_error_)/delta_t;
        prev_error_ = error;
        double angle = kp_ * error + kd_ * error_derivative + ki_ * error_integral_;
        drive_msg->drive.steering_angle = angle;
        
        //RCLCPP_INFO(get_logger(), "error = %f", error);
        //RCLCPP_INFO(get_logger(), "angle = %f", angle);
        
        if (abs(angle) > deg2rad(20.0)) {
            drive_msg->drive.speed = 1.5;
        } else if (abs(angle) > deg2rad(10.0)) {
            drive_msg->drive.speed = 1.75;
        } else {
            drive_msg->drive.speed = 2.0;
        }

        if (is_dead_end_) {
            drive_msg->drive.steering_angle = -1.5;
            drive_msg->drive.speed = 1.5;
        }

        drive_pub_->publish(*drive_msg);

    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        if (first_scan_read_) {
            angle_min_ = scan_msg->angle_min;
            angle_range_ = scan_msg->angle_max - angle_min_;
            ranges_size_ = scan_msg->ranges.size();
            left_index_ = (uint32_t)round(ranges_size_*(M_PI_2-angle_min_)/angle_range_);
            RCLCPP_INFO(get_logger(), "angle_min = %f", angle_min_);
            RCLCPP_INFO(get_logger(), "angle_range = %f", angle_range_);
            RCLCPP_INFO(get_logger(), "ranges_size = %d", ranges_size_);
            RCLCPP_INFO(get_logger(), "left_index = %d", left_index_);
            first_scan_read_ = false;
        }

        const float* ranges = scan_msg->ranges.data();

        // Get two lidar readings at forward left and forward right
        double range_front_left = get_range(ranges, deg2rad(theta_front_left_));
        double range_front_right = get_range(ranges, deg2rad(theta_front_right_));


        double range_diff = range_front_right - range_front_left;
        is_dead_end_ = false;

        if (range_diff > 1 && range_front_left < 3) {
            RCLCPP_INFO(get_logger(), "range_front_left = %f", range_front_left);
            RCLCPP_INFO(get_logger(), "range_front_right = %f", range_front_right);
            is_dead_end_ = true;
        } else {
            is_dead_end_ = false;
        }

        pid_control(get_error(scan_msg->ranges.data(), DIST));
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}