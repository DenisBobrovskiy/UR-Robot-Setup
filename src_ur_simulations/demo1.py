#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        self.get_logger().info('Robot mover ready!')

    def move_to(self, positions, duration_sec=3):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=duration_sec, nanosec=0)
        msg.points.append(point)
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Moving to: {positions}')


def main():
    rclpy.init()
    node = RobotMover()
    
    # Wait for controllers to be ready
    time.sleep(2)
    
    positions = [
        # [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],    # Home
        [1.0, -1.2, 1.2, -1.0, -1.57, 0.5],       # Position A
        [-1.0, -1.8, 1.8, -1.57, -1.57, -0.5],    # Position B
        [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],    # Back home
    ]
    
    for pos in positions:
        node.move_to(pos, duration_sec=3)
        time.sleep(4)  # Wait for move to complete
    
    node.get_logger().info('Done!')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
