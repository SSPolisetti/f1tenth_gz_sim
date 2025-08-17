#ifndef ACKERMANN_VEHICLE_CONTROLLER_HPP
#define ACKERMANN_VEHICLE_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace f1tenth_control {
    class AckermannVehicleController : public rclcpp::Node {
        public:
            AckermannVehicleController();
            
            void ackermann_drive_callback(ackermann_msgs::msg::AckermannDriveStamped msg);
            void timer_callback();
            
            std::vector<double> calculate_ackermann_steering();
            std::vector<double> calculate_ackermann_traction();
    
        private:
            rclcpp::TimerBase::SharedPtr timer_;
            double steering_angle_;
            double steering_angle_velocity_;
            double speed_;
            double accel_;
            double jerk_;
            double max_steering_angle_;
            double max_steering_angle_velocity_;
            double max_speed_;
            double max_accel_;
            double max_jerk_;
            double wheelbase_;
            double track_width_;
            double ackermann_percentage_;

            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_positions_pub_;
            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr traction_velocities_pub_;
            rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_drive_sub_;
    };
}

#endif // ACKERMANN_VEHICLE_CONTROLLER_HPP