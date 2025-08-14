import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def generate_launch_description():
    urdf_file_name = 'f1tenth_model.urdf.xacro'
    path_to_urdf = os.path.join(
        FindPackageShare('f1tenth_gz_sim').find('f1tenth_gz_sim'),
        'model',
        urdf_file_name
    )

    robot_controllers_config = PathJoinSubstitution(
        [
            FindPackageShare("f1tenth_gz_sim"),
            "config",
            "vehicle_controllers.yaml"
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("f1tenth_gz_sim"),
            "config",
            "f1tenth_model.rviz"
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", rviz_config_file]

    )


    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers_config],
        output="both",
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    ackermann_vehicle_params_file = PathJoinSubstitution(
        FindPackageShare("f1tenth_gz_sim"),
        "config",
        "parameters.yaml"
    )

    ackermann_vehicle_node = Node(
        package="f1tenth_gz_sim",
        executable="ackermann_vehicle_controller",
        ros_arguments=["--param-file", ]
    )

    robot_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_controller",
            "velocity_controller"
            "--param-file",
            robot_controllers_config
        ]
    )


    map_config_path = os.path.join(
            FindPackageShare('f1tenth_gz_sim').find('f1tenth_gz_sim'),
            'config',
            "map.yaml"
    )

    with open(map_config_path, 'r') as f:
        map_config = yaml.safe_load(f)
    world = "default"

    x = map_config["x"]
    y = map_config["y"]
    z = 0;
    R = 0
    P = 0;
    Y = map_config["yaw"]
    entity_name = "f1tenth_model"
    topic = "robot_description"
    
    
    gz_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    'gz_server.launch.py'
                ]
            )
        ),
        launch_arguments=[
            ('gz_args', [world, '-v 4', '-r']),
            ('on_exit_shutdown', 'True')
        ]
    )


    spawn_gz_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    'gz_spawn_model.launch.py'
                ]
            )
        ),
        launch_arguments=[
            ("topic", topic),
            ("entity_name", entity_name),
            ("x", x),
            ("y", y),
            ("z", z),
            ("roll", R),
            ("pitch", P),
            ("yaw", Y),
            ("allow_renaming", "false")
        ]
    )



    return LaunchDescription([

        control_node,
        robot_state_pub_node,
        gz_server_launch,
        spawn_gz_model_launch,

        # delay the joint state broadcaster from running until after the model is spawned in gazebo
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_gz_model_launch,
                on_exit=[joint_state_broadcaster_spawner]
            )
        ),

        # delay rviz and the controllers from starting until after the joint_state_broadcaster has been spawned
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[rviz_node, robot_controllers_spawner]
            )
        )

    ])




