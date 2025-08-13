#include <chrono>
#include <functional>
#include <string>

#include "f1tenth_gz_sim/ackermann_vehicle_controller.hpp"

f1tenth_control::AckermannVehicleController::AckermannVehicleController() 
    : Node("ackermann_vehicle_controller")
{

    this->declare_parameter<std::string>("steering_publish_topic");
    this->declare_parameter<std::string>("traction_publish_topic");

    
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
        std::chrono::duration<double>(100),
        std::bind(&AckermannVehicleController::timer_callback, this)
    );
}


void f1tenth_control::AckermannVehicleController::ackermann_drive_callback(
    ackermann_msgs::msg::AckermannDriveStamped msg) 
{
    steering_angle_ = msg.drive.steering_angle;
    steering_angle_velocity_ = msg.drive.steering_angle_velocity;
    speed_ = msg.drive.speed;
    accel_ = msg.drive.acceleration;
    jerk_ = msg.drive.jerk;
}

std::pair<double,double> f1tenth_control::AckermannVehicleController::calculate_ackermann_steering()
{
    // put in ackermann geometry math later
    return std::make_pair(steering_angle_, steering_angle_);
}

std::pair<double,double> f1tenth_control::AckermannVehicleController::calculate_ackermann_traction()
{
    // put in ackermann geometry math later
    return std::make_pair(speed_, speed_);
}

void f1tenth_control::AckermannVehicleController::timer_callback()
{

}


