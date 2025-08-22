#include <chrono>
#include <cmath>

#include "f1tenth_gz_sim/ackermann_vehicle_controller.hpp"

#define DIAMETER(a ) ((2) * (M_PI) * )

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
    this->declare_parameter<double>("maximum_steering_angle");
    this->declare_parameter<double>("maximum_steering_angle_velocity");
    this->declare_parameter<double>("maximum_speed");
    this->declare_parameter<double>("maximum_acceleration");
    this->declare_parameter<double>("maximum_jerk");
    this->declare_parameter<double>("knuckle_radius");
    this->declare_parameter<double>("arm_length");
    this->declare_parameter<double>("chassis_width");
    this->declare_parameter<double>("wheel_length");
    this->declare_parameter<double>("wheel_radius");

    max_steering_angle_ = this->get_parameter("maximum_steering_angle").as_double();
    max_steering_angle_velocity_ = this->get_parameter("maximum_steering_angle_velocity").as_double();
    max_speed_ = this->get_parameter("maximum_speed").as_double();
    max_accel_ = this->get_parameter("maximum_acceleration").as_double();
    max_jerk_ = this->get_parameter("maximum_jerk").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    knuckle_radius_ = this->get_parameter("knuckle_radius").as_double();
    arm_length_ = this->get_parameter("arm_length").as_double();
    chassis_width_ = this->get_parameter("chassis_width").as_double();
    wheel_length_ = this->get_parameter("wheel_length").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();


    kp_track_width_ = chassis_width_ + 2 * arm_length_ - 2 * knuckle_radius_; // 2 * (arm_length_ - 2 * knuckle_radius_) + 2 * knuckle_radius_;

    track_width_ = chassis_width_ + 2 * arm_length_ + wheel_length_; // 2 * arm_length + 2 * 0.5 * wheel_length_; 

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
    if (abs(steering_angle_) > 0.00001) {

        double outer_angle = atan(
            (wheelbase_ * tan(steering_angle_)) / (wheelbase_ + 0.5 * kp_track_width_ * tan(steering_angle_)));

        double inner_angle = atan(
            (wheelbase_ * tan(steering_angle_)) / (wheelbase_ - 0.5 * kp_track_width_ * tan(steering_angle_)));
        
        // positive steering angle means turn LEFT
        if (steering_angle_ > 0.0) {
            return {inner_angle, outer_angle};
        } else {
            return {outer_angle, inner_angle};
        }
    } 
    return {0.0, 0.0};

}

std::vector<double> f1tenth_control::AckermannVehicleController::calculate_ackermann_traction()
{
    // calculate differential for all 4 wheels

    // we calculate the different wheel speeds based on the radii of the circles traced by each wheel
    // the angular velocity is derived from the angular velocity of the center of the rear axle along the circle it traces

    if (abs(steering_angle_) > 0.00001) {
        double turn_radius = wheelbase_ / tan(steering_angle_);
    
        double vehicle_angular_velocity = speed_ / turn_radius; 
        
        
        
        double rear_inner_radius = (turn_radius - 0.5 * track_width_);
        double rear_outer_radius = (turn_radius + 0.5 * track_width_);
        
        // need to account steering knuckle when determining radius of circle traced by steering tires
        double knuckle_to_wheel_center = knuckle_radius_ + 0.5 * wheel_length_;
        
        // wheelbase and radius from rear axle are used to calculate hypotenuse, which is the radius of the circle traced by steering tires
        double front_inner_radius = sqrt(pow(wheelbase_,2) + pow(turn_radius - 0.5 * kp_track_width_,2)) - knuckle_to_wheel_center;
        double front_outer_radius = sqrt(pow(wheelbase_,2) + pow(turn_radius + 0.5 * kp_track_width_,2)) + knuckle_to_wheel_center;
        
        //rear differential
        double back_outer_velocity = rear_outer_radius * vehicle_angular_velocity;
        double back_inner_velocity = rear_inner_radius * vehicle_angular_velocity;
        
        // front differential
        double front_outer_velocity = front_outer_radius * vehicle_angular_velocity;
        double front_inner_velocity = front_inner_radius * vehicle_angular_velocity;
        
        
        // positive steering angle means turn LEFT
        if (steering_angle_ > 0.0) {
            return {front_inner_velocity, front_outer_velocity, back_inner_velocity, back_outer_velocity};
        } else {
            return {front_outer_velocity, front_inner_velocity, back_outer_velocity, back_inner_velocity};
        }
    } 
    return {speed_, speed_ , speed_, speed_};

}

void f1tenth_control::AckermannVehicleController::timer_callback()
{
    auto new_angles = calculate_ackermann_steering();
    auto new_velocities = calculate_ackermann_traction();

    for (double& vel : new_velocities) {
        vel /= wheel_radius_;
    }

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