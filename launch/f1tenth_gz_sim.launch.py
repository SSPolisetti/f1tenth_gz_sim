
import os
import yaml
import xacro
import xml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_share_dir = get_package_share_directory('f1tenth_gz_sim')

    ros_params_file = os.path.join(
        package_share_dir,
        "config", 
        "parameters.yaml"
    )

    car_description_path = os.path.join(package_share_dir, "model", "f1tenth_car.urdf.xacro")    

    with open(ros_params_file, "r") as f:
        car_arguments = yaml.safe_load(f)["/**"]["ros__parameters"]
    
    sim_config_path = os.path.join(
        package_share_dir,
        'config',
        "sim.yaml"
    )


    with open(sim_config_path, "r") as f:
        sim_config = yaml.safe_load(f)
    
    track = sim_config["track"]

    track_config_path = os.path.join(package_share_dir, "world", "tracks", track, f"{track}.yaml")

    with open(track_config_path, "r") as f:
        track_config = yaml.safe_load(f)

    world = os.path.join(package_share_dir, "world", "tracks", track, f"{track}.sdf")
    x = str(track_config["x"])
    y = str(track_config["y"])
    z = str(0.0781)
    # R = "0"
    # P = "0"
    Y = str(track_config["yaw"])
    entity_name = "f1tenth_car"
    topic = "robot_description"

    print({key: str(value) for key, value in car_arguments.items()})

    f1tenth_car_description = xacro.process(car_description_path, mappings={key: str(value) for key, value in car_arguments.items()})

    # robot_controllers_config = PathJoinSubstitution([
    #     FindPackageShare("f1tenth_gz_sim"),
    #     "config",
    #     "vehicle_controllers.yaml"
    # ])

    robot_controllers_config = os.path.join(
        package_share_dir,
        "config",
        "vehicle_controllers.yaml"
    )

    # rviz_config_file = PathJoinSubstitution([
    #     FindPackageShare("f1tenth_gz_sim"),
    #     "config",
    #     "f1tenth_model.rviz"
    # ])

    rviz_config_file = os.path.join(
        package_share_dir,
        "rviz",
        "f1tenth_model.rviz"
    )

    bridge_config_file = os.path.join(
        package_share_dir,
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
            'robot_description': f1tenth_car_description,
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

    # ackermann_vehicle_params_file = PathJoinSubstitution([
    #     FindPackageShare("f1tenth_gz_sim"),
    #     "config",
    #     "parameters.yaml"
    # ])


    ackermann_vehicle_node = Node(
        package="f1tenth_gz_sim",
        executable="ackermann_vehicle_controller",
        parameters=[ros_params_file]
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
            # "-R", R,
            # "-P", P,
            "-Y", Y,
            "-allow_renaming", "false"
        ]
    )
    
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controllers_spawner,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )
    
    return LaunchDescription([
        robot_state_pub_node,
        gz_sim_launch,
        spawn_gz_model_node,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        robot_controllers_spawner,
        ackermann_vehicle_node,
        # rviz_node,
        ros_gz_bridge_node
    ])




