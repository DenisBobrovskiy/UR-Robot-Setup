#!/usr/bin/env python3
"""
Simple UR Robot Demo - Relative Movements
Gets current position, then moves left and right relative to it.
"""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time
from threading import Event


class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        
        # Current joint positions (updated by subscriber)
        self.current_positions = None
        self.position_received = Event()
        
        # Joint names for UR robots
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Publisher for movement commands
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Subscriber to get current joint positions
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info('Robot mover initialized, waiting for joint states...')

    def joint_state_callback(self, msg: JointState):
        """Store current joint positions when received"""
        # Create a dict to map name -> position
        positions_dict = dict(zip(msg.name, msg.position))
        
        # Extract positions in the correct order
        try:
            self.current_positions = [
                positions_dict[name] for name in self.joint_names
            ]
            self.position_received.set()
        except KeyError as e:
            self.get_logger().warn(f'Missing joint in state: {e}')

    def wait_for_position(self, timeout=5.0) -> bool:
        """Wait until we receive current position"""
        self.position_received.clear()
        
        # Spin until we get position or timeout
        start = time.time()
        while not self.position_received.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                self.get_logger().error('Timeout waiting for joint states')
                return False
        return True

    def get_current_positions(self) -> list:
        """Get current joint positions"""
        if self.current_positions is None:
            self.wait_for_position()
        return list(self.current_positions) if self.current_positions else None

    def move_to(self, positions: list, duration_sec: float = 2.0):
        """Move to absolute joint positions"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        msg.points.append(point)
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Moving to: {[f"{p:.3f}" for p in positions]}')

    def move_relative(self, deltas: list, duration_sec: float = 2.0):
        """
        Move relative to current position.
        
        Args:
            deltas: List of 6 values to add to current positions (radians)
            duration_sec: Time to complete move
        """
        current = self.get_current_positions()
        if current is None:
            self.get_logger().error('Cannot get current position')
            return
        
        # Calculate new target
        target = [curr + delta for curr, delta in zip(current, deltas)]
        
        self.get_logger().info(f'Current: {[f"{p:.3f}" for p in current]}')
        self.get_logger().info(f'Delta:   {[f"{d:.3f}" for d in deltas]}')
        
        self.move_to(target, duration_sec)


def main():
    rclpy.init()
    node = RobotMover()
    
    # Wait for controllers and joint states to be ready
    node.get_logger().info('Waiting for system to be ready...')
    time.sleep(2)
    
    # Get and print current position
    if not node.wait_for_position():
        node.get_logger().error('Failed to get initial position')
        return
    
    current = node.get_current_positions()
    node.get_logger().info('=== Current Joint Positions ===')
    for name, pos in zip(node.joint_names, current):
        node.get_logger().info(f'  {name}: {pos:.4f} rad ({pos * 57.2958:.2f} deg)')
    
    # Small movement amount (radians)
    # 0.2 rad ≈ 11.5 degrees
    small_move = 0.2
    
    # Define relative movements
    # [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
    movements = [
        # Move shoulder left (positive rotation)
        ([small_move, 0, 0, 0, 0, 0], "Shoulder LEFT"),
        
        # Move shoulder right (back to center, then past it)
        ([-small_move * 2, 0, 0, 0, 0, 0], "Shoulder RIGHT"),
        
        # Back to center
        ([small_move, 0, 0, 0, 0, 0], "Back to CENTER"),
        
        # Wrist rotation
        ([0, 0, 0, 0, 0, small_move], "Wrist rotate LEFT"),
        ([0, 0, 0, 0, 0, -small_move], "Wrist rotate RIGHT (back)"),
    ]
    
    node.get_logger().info('=== Starting Movement Demo ===')
    
    for delta, description in movements:
        node.get_logger().info(f'\n>>> {description}')
        node.move_relative(delta, duration_sec=2.0)
        time.sleep(2.5)  # Wait for move to complete
        
        # Update current position after move
        rclpy.spin_once(node, timeout_sec=0.5)
    
    node.get_logger().info('=== Demo Complete ===')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()