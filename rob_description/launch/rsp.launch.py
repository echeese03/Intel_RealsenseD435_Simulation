from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    controllers_config = PathJoinSubstitution([FindPackageShare("rsp_week05"), "config", "control.yaml"])
    
    robot_description_content = Command(
    [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("rsp_week05"), "urdf", "rsp.urdf.xacro"]),
        " ",
        "use_simulation:=false",
        " ",
        "use_fake_hardware:=true",
        " ",
        "sim_controllers:=",
        controllers_config,
    ]
    )
    robot_description = {"robot_description": robot_description_content}

    ignition_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": " -r -v 3 "}.items(),
    )

    ignition_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
        ],
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_config],
        output="screen"
    )
    
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen"
    )
    
    joint_command_position = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "-c", "/controller_manager"],
        output="screen"
    )

    joint_command_velocity = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "-c", "/controller_manager", "--inactive"],
        output="screen"
    )

    linear_trajectory = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["linear_trajectory_controller", "-c", "/controller_manager", "--inactive"],
        output="screen"
    )
    
    return LaunchDescription(
        [
            ignition_launch_description,
            ignition_spawn_robot,
            robot_state_publisher_node,
            # control_node,
            joint_state_broadcaster,
            joint_command_position,
            joint_command_velocity,
            linear_trajectory
        ]
    )

