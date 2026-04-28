from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os

def generate_launch_description():
    nodes = []
    config_dir = os.path.expanduser('~/swarm_ws/src/swarm_localization/config')

    # Publish world frame as static transform
    world_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_frame_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )
    nodes.append(world_frame)

    # EKF nodes for all robots
    for i in range(1, 5):
        ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name=f'robot_{i}_ekf',
            namespace=f'robot_{i}',
            parameters=[
                os.path.join(config_dir, f'robot_{i}_ekf.yaml'),
                {'use_sim_time': True}
            ],
            remappings=[
                ('odometry/filtered', f'/robot_{i}/odom/filtered')
            ]
        )
        nodes.append(ekf_node)

    # Auto set initial poses after 5 seconds
    set_poses = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='swarm_localization',
                executable='set_initial_poses',
                name='initial_pose_setter',
            )
        ]
    )
    nodes.append(set_poses)

    return LaunchDescription(nodes)
