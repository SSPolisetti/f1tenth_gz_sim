import os
import yaml
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_share_dir = get_package_share_directory("f1tenth_gz_sim")

    ros_params_file = os.path.join(
        package_share_dir,
        "config", 
        "parameters.yaml"
    )
    car_description_path = os.path.join(package_share_dir, "model", "f1tenth_car.urdf.xacro")    


    with open(ros_params_file, "r") as f:
        car_arguments = yaml.safe_load(f)["/**"]["ros__parameters"]

    f1tenth_car_description = xacro.process(car_description_path, mappings={key: str(value) for key, value in car_arguments.items()})

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': f1tenth_car_description
            }],
        ),
        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])