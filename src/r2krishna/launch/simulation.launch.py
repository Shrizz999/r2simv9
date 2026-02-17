import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_r2krishna = get_package_share_directory('r2krishna')
    pkg_arena_viz = get_package_share_directory('arena_viz')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Resource paths
    install_dir = os.path.dirname(pkg_r2krishna)
    gz_resource_path = f"{install_dir}:{pkg_arena_viz}/models"
    
    urdf_file = os.path.join(pkg_r2krishna, 'urdf', 'auto_3.urdf')
    world_file = os.path.join(pkg_arena_viz, 'worlds', 'arena.world')
    rviz_config = os.path.join(pkg_arena_viz, 'rviz', 'arena.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_resource_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description', 
                '-name', 'auto_3', 
                '-x', '-1.0',  # CHANGED: X is now 1.0
                '-y', '0.5',  # CHANGED: Y is now 8.0
                '-z', '1.0',
                '-Y', '3.14'
            ],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'qos_overrides./tf_static.publisher.durability': 'transient_local'}],
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
            ],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        )
    ])
