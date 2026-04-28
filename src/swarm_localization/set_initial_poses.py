#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from robot_localization.srv import SetPose
from nav_msgs.msg import Odometry
import time

COVARIANCE = [
    0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.1
]

class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__(
            'initial_pose_setter',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ]
        )
        self.get_logger().info('Reading positions from odom topics...')
        self.robots = ['robot_1', 'robot_2', 'robot_3', 'robot_4']
        self.positions = {}

        # Subscribe to each robot's odom
        for robot in self.robots:
            self.create_subscription(
                Odometry,
                f'/{robot}/odom',
                lambda msg, r=robot: self.odom_callback(msg, r),
                10
            )

        # Spin until we have all positions
        self.get_logger().info('Waiting for odom data...')
        start = time.time()
        while len(self.positions) < len(self.robots):
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() - start > 15.0:
                self.get_logger().warn('Timeout!')
                break

        self.get_logger().info(f'Got {len(self.positions)} positions')

        # Now set initial poses
        for robot, pos in self.positions.items():
            self.set_pose(robot, pos)

    def odom_callback(self, msg, robot_name):
        if robot_name not in self.positions:
            # Get the world position from the odom message header
            # The odom frame_id tells us the reference frame
            p = msg.pose.pose.position
            self.positions[robot_name] = {
                'x': p.x, 'y': p.y, 'z': p.z,
                'frame': msg.header.frame_id
            }
            self.get_logger().info(
                f'{robot_name}: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) '
                f'frame: {msg.header.frame_id}')

    def set_pose(self, robot_name, pos):
        client = self.create_client(SetPose, f'/{robot_name}/set_pose')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service not available for {robot_name}')
            return

        request = SetPose.Request()
        request.pose.header.frame_id = pos.get('frame', 'world')
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.pose.position.x = float(pos['x'])
        request.pose.pose.pose.position.y = float(pos['y'])
        request.pose.pose.pose.position.z = float(pos['z'])
        request.pose.pose.pose.orientation.w = 1.0
        request.pose.pose.covariance = [float(x) for x in COVARIANCE]

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            self.get_logger().info(f'✅ {robot_name} pose set!')
        else:
            self.get_logger().error(f'❌ Failed for {robot_name}')

def main():
    rclpy.init()
    node = InitialPoseSetter()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
