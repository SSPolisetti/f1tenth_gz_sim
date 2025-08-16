import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import yaml


def generate_launch_description():
    urdf_file_name = 'f1tenth_model.urdf.xacro'
    path_to_urdf = os.path.join(
        FindPackageShare('f1tenth_gz_sim').find('f1tenth_gz_sim'),
        'model',
        urdf_file_name
    )

    robot_controllers_config = PathJoinSubstitution([
        FindPackageShare("f1tenth_gz_sim"),
        "config",
        "vehicle_controllers.yaml"
    ])


    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("f1tenth_gz_sim"),
        "config",
        "f1tenth_model.rviz"
    ])


    bridge_config_file = os.path.join(
        get_package_share_directory("f1tenth_gz_sim"),
        "config",
        "bridge_parameters.yaml"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", rviz_config_file]

    )
        
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', str(path_to_urdf)]), value_type=str
            ),
            'use_sim_time': True
        }],
    )

    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_config_file}"]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    ackermann_vehicle_params_file = PathJoinSubstitution([
        FindPackageShare("f1tenth_gz_sim"),
        "config",
        "parameters.yaml"
    ])

    ackermann_vehicle_node = Node(
        package="f1tenth_gz_sim",
        executable="ackermann_vehicle_controller",
        parameters=[ackermann_vehicle_params_file]
    )

    robot_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_controller",
            "velocity_controller",
            "--param-file",
            robot_controllers_config
        ]
    )


    sim_config_path = os.path.join(
            FindPackageShare('f1tenth_gz_sim').find('f1tenth_gz_sim'),
            'config',
            "sim.yaml"
    )

    with open(sim_config_path, 'r') as f:
        sim_config = yaml.safe_load(f)

    world = "empty.sdf"
    x = str(sim_config["x"])
    y = str(sim_config["y"])
    z = str(sim_config["z"])
    R = "0"
    P = "0"
    Y = str(sim_config["yaw"])
    entity_name = "f1tenth_model"
    topic = "robot_description"
    
    
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments=[
            ('gz_args', ['-v 4 -r ' + world]),
            ('on_exit_shutdown', 'True')
        ]
    )


    spawn_gz_model_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", topic,
            "-entity_name", entity_name,
            "-x", x,
            "-y", y,
            "-z", z,
            "-R", R,
            "-P", P,
            "-Y", Y,
            "-allow_renaming", "false"
        ]
        
    )
    
    return LaunchDescription([

        robot_state_pub_node,
        gz_sim_launch,
        spawn_gz_model_node,
        joint_state_broadcaster_spawner,
        robot_controllers_spawner,
        rviz_node,
        ros_gz_bridge_node
        # delay the joint state broadcaster from running until after the model is spawned in gazebo
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawn_gz_model_node,
        #         on_exit=[joint_state_broadcaster_spawner]
        #     )
        # ),

        # # delay rviz and the controllers from starting until after the joint_state_broadcaster has been spawned
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=joint_state_broadcaster_spawner,
        #         on_exit=[rviz_node, robot_controllers_spawner]
        #     )
        # ),
        

    ])




