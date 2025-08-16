import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue




def generate_launch_description():

    urdf_file_name = 'f1tenth_model.urdf.xacro'
    path_to_urdf = os.path.join(
        FindPackageShare('f1tenth_gz_sim').find('f1tenth_gz_sim'),
        'model',
        urdf_file_name
    )
    
    # with open(path_to_urdf, 'r') as f:
    #     urdf_file = f.read()

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', str(path_to_urdf)]), value_type=str
                )
            }],
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])