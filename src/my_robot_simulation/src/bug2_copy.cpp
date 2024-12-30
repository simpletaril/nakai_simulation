#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class PlaceholderController : public rclcpp::Node
{
public:
    PlaceholderController()
        : Node("placeholder_controller"),
        left_dist(999999.99), leftfront_dist(999999.99),
        front_dist(999999.99), rightfront_dist(999999.99),
        right_dist(999999.99), forward_speed(0.035),
        current_x(0.0), current_y(0.0), current_yaw(0.0),
        robot_mode("go to goal mode"), dist_thresh_obs(0.25),
        turning_speed(0.25), goal_idx(0), goal_max_idx(-1),
        yaw_precision(2.0 * (M_PI/180)), turning_speed_yaw_adjustment(0.0625),
        dist_precision(0.2), wall_following_state("turn left"),
        turning_speed_wf_fast(1.0), turning_speed_wf_slow(0.125),
        dist_thresh_wf(0.45), dist_too_close_to_wall(0.15),
        bug2_switch("ON"), start_goal_line_calculated(false),
        dist_thresh_bug2(0.15), distance_to_start_goal_line_precision(0.1),
        leave_point_to_hit_point_diff(0.25)
    {

        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/en613/state_est", 10, std::bind(&PlaceholderController::state_estimate_callback,
            this, std::placeholders::_1));
        
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/en613/scan",
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), std::bind(&PlaceholderController::scan_callback,
        this, std::placeholders::_1));

        subscription_goal_pose_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/en613/goal", 10, std::bind(&PlaceholderController::pose_recieved, this, std::placeholders::_1)
        );


        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    }

private:

    // Member variables
    double left_dist, leftfront_dist, front_dist, rightfront_dist, right_dist;
    double forward_speed, current_x, current_y, current_yaw;
    std::string robot_mode;
    double dist_thresh_obs, turning_speed;
    std::string go_to_goal_state;
    std::vector<double> goal_x_coordinates, goal_y_coordinates;
    int goal_idx, goal_max_idx;
    double yaw_precision, turning_speed_yaw_adjustment, dist_precision;
    std::string wall_following_state;
    double turning_speed_wf_fast, turning_speed_wf_slow;
    double dist_thresh_wf, dist_too_close_to_wall;
    std::string bug2_switch;
    bool start_goal_line_calculated;
    double start_goal_line_slope_m, start_goal_line_y_intercept;
    double start_goal_line_xstart, start_goal_line_xgoal;
    double start_goal_line_ystart, start_goal_line_ygoal;
    double dist_thresh_bug2, distance_to_start_goal_line_precision;
    double hit_point_x, hit_point_y, distance_to_goal_from_hit_point;
    double leave_point_x, leave_point_y, distance_to_goal_from_leave_point;
    double leave_point_to_hit_point_diff;


    void pose_recieved(const geometry_msgs::msg::Pose::SharedPtr msg){
        goal_x_coordinates.clear();
        goal_y_coordinates.clear();
        goal_x_coordinates.push_back(msg->position.x);
        goal_y_coordinates.push_back(msg->position.y);
        goal_max_idx = goal_x_coordinates.size() - 1;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        left_dist = msg->ranges[180];
        leftfront_dist = msg->ranges[135];
        front_dist = msg->ranges[90];
        rightfront_dist = msg->ranges[45];
        right_dist = msg->ranges[0];

        if(robot_mode = "obstacle avoidance mode")
        {
            aviod_obstacles();
        }
    }

    void state_estimate_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> curr_state = msg->data;
    }


    




    void aviod_obstacles(){
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x= 0.0;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;

        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;

        double d = dist_thresh_obs;

    }



    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_goal_pose_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};