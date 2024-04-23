import launch
import launch_ros
import os

def generate_launch_description():
    package_name = 'rhcr-slam-nav2'
    pkg_share = launch_ros.substitutions.FindPackageShare(package=package_name).find(package_name)
    model_path = os.path.join(pkg_share, 'src/description/unity_ackermann_description.urdf')
    slam_config_path = os.path.join(pkg_share, 'config/slam.config.yaml')
    nav2_config_path = os.path.join(pkg_share, 'config/nav2.config.yaml')
    rviz_config_path = os.path.join(pkg_share, 'config/nav2.config.rviz')
    nav2_pkg_share = launch_ros.substitutions.FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    nav2_launch_path = os.path.join(nav2_pkg_share, 'launch/navigation_launch.py')

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
    navigation_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={'params_file': nav2_config_path}.items()
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return launch.LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        ackermann_node,
        tf2_node,
        slam_node,
        navigation_launch,
        rviz_node
    ])