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
        : Node("PlaceholderController"),
          left_dist(999999.9), leftfront_dist(999999.9),
          front_dist(999999.9), rightfront_dist(999999.9),
          right_dist(999999.9), forward_speed(0.035),
          current_x(0.0), current_y(0.0), current_yaw(0.0),
          robot_mode("go to goal mode"), dist_thresh_obs(0.25),
          turning_speed(0.25), goal_idx(0), goal_max_idx(-1),
          yaw_precision(2.0 * (M_PI / 180)), turning_speed_yaw_adjustment(0.0625),
          dist_precision(0.2), wall_following_state("turn left"),
          turning_speed_wf_fast(1.0), turning_speed_wf_slow(0.125),
          dist_thresh_wf(0.45), dist_too_close_to_wall(0.15),
          bug2_switch("ON"), start_goal_line_calculated(false),
          dist_thresh_bug2(0.15), distance_to_start_goal_line_precision(0.1),
          leave_point_to_hit_point_diff(0.25)
    {
        // Subscribers
        subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/en613/state_est", 10,
            std::bind(&PlaceholderController::state_estimate_callback, this, std::placeholders::_1));

        scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/en613/scan", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
            std::bind(&PlaceholderController::scan_callback, this, std::placeholders::_1));

        subscription_goal_pose = this->create_subscription<geometry_msgs::msg::Pose>(
            "/en613/goal", 10,
            std::bind(&PlaceholderController::pose_received, this, std::placeholders::_1));

        // Publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
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

    // Subscribers and Publisher
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_goal_pose;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    // Callback for the pose received
    void pose_received(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        goal_x_coordinates.clear();
        goal_y_coordinates.clear();
        goal_x_coordinates.push_back(msg->position.x);
        goal_y_coordinates.push_back(msg->position.y);
        goal_max_idx = goal_x_coordinates.size() - 1;
    }

    // Callback for the scan data received
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        left_dist = msg->ranges[180];
        leftfront_dist = msg->ranges[135];
        front_dist = msg->ranges[90];
        rightfront_dist = msg->ranges[45];
        right_dist = msg->ranges[0];

        if (robot_mode == "obstacle avoidance mode")
        {
            avoid_obstacles();
        }
    }

    // Callback for state estimate
    void state_estimate_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> curr_state = msg->data;
        current_x = curr_state[0];
        current_y = curr_state[1];
        current_yaw = curr_state[2];

        if (goal_x_coordinates.empty() && goal_y_coordinates.empty())
            return;

        if (bug2_switch == "ON")
        {
            // Call bug2() here (implementation not shown)
        }
        else
        {
            if (robot_mode == "go to goal mode")
            {
                // Call go_to_goal() here (implementation not shown)
            }
            else if (robot_mode == "wall following mode")
            {
                // Call follow_wall() here (implementation not shown)
            }
        }
    }

    // Method for obstacle avoidance
    void avoid_obstacles()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;

        double d = dist_thresh_obs;

        if (leftfront_dist > d && front_dist > d && rightfront_dist > d)
        {
            // Logic for avoiding obstacles (implementation not shown)
        }
        // Continue implementing logic as needed...
    }

        void go_to_goal() {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;

        if (bug2_switch == "ON") {
            double d = dist_thresh_bug2;
            if (leftfront_dist < d || front_dist < d || rightfront_dist < d) {
                robot_mode = "wall following mode";
                hit_point_x = current_x;
                hit_point_y = current_y;

                distance_to_goal_from_hit_point = std::sqrt(
                    std::pow(goal_x_coordinates[goal_idx] - hit_point_x, 2) +
                    std::pow(goal_y_coordinates[goal_idx] - hit_point_y, 2));

                msg.angular.z = turning_speed_wf_fast;
                publisher_->publish(msg);
                return;
            }
        }

        if (go_to_goal_state == "adjust heading") {
            double desired_yaw = std::atan2(
                goal_y_coordinates[goal_idx] - current_y,
                goal_x_coordinates[goal_idx] - current_x);

            double yaw_error = desired_yaw - current_yaw;

            if (std::fabs(yaw_error) > yaw_precision) {
                msg.angular.z = (yaw_error > 0) ? turning_speed_yaw_adjustment : -turning_speed_yaw_adjustment;
                publisher_->publish(msg);
            } else {
                go_to_goal_state = "go straight";
                publisher_->publish(msg);
            }
        } else if (go_to_goal_state == "go straight") {
            double position_error = std::sqrt(
                std::pow(goal_x_coordinates[goal_idx] - current_x, 2) +
                std::pow(goal_y_coordinates[goal_idx] - current_y, 2));

            if (position_error > dist_precision) {
                msg.linear.x = forward_speed;
                publisher_->publish(msg);

                double desired_yaw = std::atan2(
                    goal_y_coordinates[goal_idx] - current_y,
                    goal_x_coordinates[goal_idx] - current_x);

                double yaw_error = desired_yaw - current_yaw;

                if (std::fabs(yaw_error) > yaw_precision) {
                    go_to_goal_state = "adjust heading";
                }
            } else {
                go_to_goal_state = "goal achieved";
                publisher_->publish(msg);
            }
        } else if (go_to_goal_state == "goal achieved") {
            RCLCPP_INFO(this->get_logger(), "Goal achieved! X: %f Y: %f",
                         goal_x_coordinates[goal_idx], goal_y_coordinates[goal_idx]);

            goal_idx++;

            if (goal_idx > goal_max_idx) {
                RCLCPP_INFO(this->get_logger(), "Congratulations! All goals have been achieved.");
                rclcpp::shutdown();
            } else {
                go_to_goal_state = "adjust heading";
                start_goal_line_calculated = false;  // Reset for the next goal
            }
        }
    }

    void follow_wall() {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;

        if (bug2_switch == "ON") {
            double x_start_goal_line = current_x;
            double y_start_goal_line = start_goal_line_slope_m * x_start_goal_line + start_goal_line_y_intercept;

            double distance_to_start_goal_line = std::sqrt(
                std::pow(x_start_goal_line - current_x, 2) +
                std::pow(y_start_goal_line - current_y, 2));

            if (distance_to_start_goal_line < distance_to_start_goal_line_precision) {
                leave_point_x = current_x;
                leave_point_y = current_y;

                distance_to_goal_from_leave_point = std::sqrt(
                    std::pow(goal_x_coordinates[goal_idx] - leave_point_x, 2) +
                    std::pow(goal_y_coordinates[goal_idx] - leave_point_y, 2));

                double diff = distance_to_goal_from_hit_point - distance_to_goal_from_leave_point;
                if (diff > leave_point_to_hit_point_diff) {
                    robot_mode = "go to goal mode";
                }
                return;
            }
        }

        double d = dist_thresh_wf;
        if (leftfront_dist > d && front_dist > d && rightfront_dist > d) {
            wall_following_state = "search for wall";
            msg.linear.x = forward_speed;
            msg.angular.z = -turning_speed_wf_slow;
        } else if (leftfront_dist > d && front_dist < d && rightfront_dist > d) {
            wall_following_state = "turn left";
            msg.angular.z = turning_speed_wf_fast;
        } else if (leftfront_dist > d && front_dist > d && rightfront_dist < d) {
            if (rightfront_dist < dist_too_close_to_wall) {
                wall_following_state = "turn left";
                msg.linear.x = forward_speed;
                msg.angular.z = turning_speed_wf_fast;
            } else {
                wall_following_state = "follow wall";
                msg.linear.x = forward_speed;
            }
        } else if (leftfront_dist < d && front_dist > d && rightfront_dist > d) {
            wall_following_state = "search for wall";
            msg.linear.x = forward_speed;
            msg.angular.z = -turning_speed_wf_slow;
        } else if (leftfront_dist > d && front_dist < d && rightfront_dist < d) {
            wall_following_state = "turn left";
            msg.angular.z = turning_speed_wf_fast;
        } else if (leftfront_dist < d && front_dist < d && rightfront_dist > d) {
            wall_following_state = "turn left";
            msg.angular.z = turning_speed_wf_fast;
        } else if (leftfront_dist < d && front_dist < d && rightfront_dist < d) {
            wall_following_state = "turn left";
            msg.angular.z = turning_speed_wf_fast;
        } else if (leftfront_dist < d && front_dist > d && rightfront_dist < d) {
            wall_following_state = "search for wall";
            msg.linear.x = forward_speed;
            msg.angular.z = -turning_speed_wf_slow;
        }

        publisher_->publish(msg);
    }

    void bug2() {
        if (!start_goal_line_calculated) {
            robot_mode = "go to goal mode";
            start_goal_line_xstart = current_x;
            start_goal_line_xgoal = goal_x_coordinates[goal_idx];
            start_goal_line_ystart = current_y;
            start_goal_line_ygoal = goal_y_coordinates[goal_idx];

            start_goal_line_slope_m = (start_goal_line_ygoal - start_goal_line_ystart) /
                (start_goal_line_xgoal - start_goal_line_xstart);
            start_goal_line_y_intercept = start_goal_line_ygoal - 
                (start_goal_line_slope_m * start_goal_line_xgoal);
            start_goal_line_calculated = true;
        }

        if (robot_mode == "go to goal mode") {
            go_to_goal();
        } else if (robot_mode == "wall following mode") {
            follow_wall();
        }
    }
};    
    
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlaceholderController>());
    rclcpp::shutdown();
    return 0;
}
