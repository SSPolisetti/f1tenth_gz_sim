#include <chrono>
#include <cmath>

#include "f1tenth_gz_sim/ackermann_vehicle_controller.hpp"

using namespace std::chrono_literals;

f1tenth_control::AckermannVehicleController::AckermannVehicleController() 
    : Node("ackermann_vehicle_controller"),
    steering_angle_(0),
    steering_angle_velocity_(0),
    speed_(0),
    accel_(0),
    jerk_(0)
{

    this->declare_parameter<std::string>("steering_publish_topic");
    this->declare_parameter<std::string>("traction_publish_topic");
    this->declare_parameter<double>("wheelbase");
    this->declare_parameter<double>("kp_track_width");
    this->declare_parameter<double>("ackermann_percentage");
    this->declare_parameter<double>("maximum_steering_angle");
    this->declare_parameter<double>("maximum_steering_angle_velocity");
    this->declare_parameter<double>("maximum_speed");
    this->declare_parameter<double>("maximum_accel");
    this->declare_parameter<double>("maximum_jerk");

    max_steering_angle_ = this->get_parameter("maximum_steering_angle").as_double();
    max_steering_angle_velocity_ = this->get_parameter("maximum_steering_angle_velocity").as_double();
    max_speed_ = this->get_parameter("maximum_speed").as_double();
    max_accel_ = this->get_parameter("maximum_acceleration").as_double();
    max_jerk_ = this->get_parameter("maximum_jerk").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    track_width_ = this->get_parameter("track_width").as_double();
    ackermann_percentage_ = this->get_parameter("ackermann_percentage").as_double();

    steering_positions_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        this->get_parameter("steering_publish_topic").as_string(),
        10
    );
    
    traction_velocities_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        this->get_parameter("traction_publish_topic").as_string(),
        10
    );
    
    ackermann_drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/drive",
        10,
        std::bind(
            &AckermannVehicleController::ackermann_drive_callback, 
            this, 
            std::placeholders::_1
        )
    );

    timer_ = this->create_wall_timer(
        100ns,
        std::bind(&AckermannVehicleController::timer_callback, this)
    );
}


void f1tenth_control::AckermannVehicleController::ackermann_drive_callback(
    ackermann_msgs::msg::AckermannDriveStamped msg) 
{
    steering_angle_ = std::clamp(
        (double) msg.drive.steering_angle, -max_steering_angle_, max_steering_angle_);
    steering_angle_velocity_ = std::clamp(
        (double) msg.drive.steering_angle_velocity,
        -max_steering_angle_velocity_,
        max_steering_angle_velocity_
    );

    speed_ = std::clamp((double) msg.drive.speed, 0.0, max_speed_);
    accel_ = std::clamp((double) msg.drive.acceleration, -max_accel_, max_accel_);
    jerk_ = std::clamp((double) msg.drive.jerk, -max_jerk_, max_jerk_);
}

std::vector<double> f1tenth_control::AckermannVehicleController::calculate_ackermann_steering()
{
    // ackermann steering math
    
    double outer_angle = atan(
        (wheelbase_ * tan(steering_angle_)) / (wheelbase_ + 0.5 * track_width_ * tan(steering_angle_)));

    double inner_angle = atan(
        (wheelbase_ * tan(steering_angle_)) / (wheelbase_ - 0.5 * track_width_ * tan(steering_angle_)));
    
    // positive steering angle means turn LEFT
    if (steering_angle_ > 0) {
        return {inner_angle, outer_angle};
    } else {
        return {outer_angle, inner_angle};
    }

    return {steering_angle_, steering_angle_};
}

std::vector<double> f1tenth_control::AckermannVehicleController::calculate_ackermann_traction()
{
    // put in ackermann geometry math later

    
    return {speed_, speed_, speed_, speed_};
}

void f1tenth_control::AckermannVehicleController::timer_callback()
{
    auto new_angles = calculate_ackermann_steering();
    auto new_velocities = calculate_ackermann_traction();

    auto position_msg = std_msgs::msg::Float64MultiArray();
    auto velocity_msg = std_msgs::msg::Float64MultiArray();

    position_msg.data = new_angles;
    velocity_msg.data = new_velocities;

    steering_positions_pub_->publish(position_msg);
    traction_velocities_pub_->publish(velocity_msg);    
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<f1tenth_control::AckermannVehicleController>());
  rclcpp::shutdown();
  return 0;
}