import launch
import launch.launch_description_sources
import launch_ros
import os

def generate_launch_description():
    package_name = 'rhcr'
    my_pkg_share = launch_ros.substitutions.FindPackageShare(package=package_name).find(package_name)
    model_path = os.path.join(my_pkg_share, 'src/description/unity_ackermann_description.urdf')
    ros_tcp_endpoint_pkg_share = launch_ros.substitutions.FindPackageShare(package='ros_tcp_endpoint').find('ros_tcp_endpoint')
    ros_tcp_endpoint_launch_path = os.path.join(ros_tcp_endpoint_pkg_share, 'launch/endpoint.py')
    slam_config_path = os.path.join(my_pkg_share, 'config/slam.config.yaml')
    rviz_config_path = os.path.join(my_pkg_share, 'config/slam.config.rviz')

    ros_tcp_endpoint_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(ros_tcp_endpoint_launch_path)
    )
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': launch.substitutions.Command(['xacro ', model_path])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    ackermann_node = launch_ros.actions.Node(
        package=package_name,
        executable='ackermann_tf2_broadcaster',
        name='broadcaster_ackermann',
        parameters=[
            {'odom_topic_name': 'unity_odom'},
            {'odom_frame_id': 'unity_odom'},
            {'child_frame_id': 'unity_base_link'}
        ]
    )
    tf2_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'unity_map', '--child-frame-id', 'unity_odom']
    )
    slam_node = launch_ros.actions.Node(
        parameters=[
          slam_config_path,
          {'use_sim_time': False}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox'
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return launch.LaunchDescription([
        ros_tcp_endpoint_launch,
        joint_state_publisher_node,
        robot_state_publisher_node,
        ackermann_node,
        tf2_node,
        slam_node,
        rviz_node
    ])
