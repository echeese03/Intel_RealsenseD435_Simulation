from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def launch_bridges(context):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    prefix = '/'
    if robot_name != '':
        prefix = '/' + robot_name + '_'

    ignition_bridges = Node(
            package = 'ros_ign_bridge',
            executable='parameter_bridge',
            name='bridges',
            output='screen',
            namespace=robot_name,
            arguments=[
                # '/model/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                prefix + 'realsense_d435/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                prefix + 'realsense_d435/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
                prefix + 'realsense_d435/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                prefix + 'realsense_d435/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
    ])

    return [ignition_bridges]

def generate_launch_description():

    robot_name = LaunchConfiguration('robot_name')
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='rob1')
    launch_args = [
        robot_name_arg,
    ]

    robot_description_content = Command(
    [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("rob_description"), "urdf", "rob.urdf.xacro"]),
        " ",
        "name:=", robot_name,
    ]
    )
    robot_description = {"robot_description": robot_description_content}


    world = "test.sdf"
    gz_args = "-v 4 " + os.path.join(get_package_share_directory('rob_description'), 'world', world)
    ignition_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    x = "0"
    y = "0"
    z = "0.008"
    roll = "0"
    pitch = "0"
    yaw = "0"
    ignition_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            '-x', x, '-y', y, '-z', z, 
            '-R', roll, '-P', pitch, '-Y', yaw
        ],
    )

    ignition_bridges = OpaqueFunction(function = launch_bridges)
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        output="both",
        parameters=[robot_description],
    )
    
    return LaunchDescription(
        launch_args + 
        [
            ignition_launch_description,
            ignition_spawn_robot,
            ignition_bridges,
            robot_state_publisher,
        ]
    )

