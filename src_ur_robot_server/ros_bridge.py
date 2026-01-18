"""ROS2 communication bridge."""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from builtin_interfaces.msg import Duration
from threading import Thread
import time

from config import ROBOT_CONFIG, SERVER_CONFIG
from state_manager import StateManager


class ROSBridge(Node):
    """ROS2 node that bridges to the robot."""
    
    def __init__(self, state_manager: StateManager):
        super().__init__('ur_control_bridge')
        
        self.state_manager = state_manager
        self.joint_names = ROBOT_CONFIG.joint_names
        self._received_joint_state = False
        
        # Publisher for trajectory commands
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            SERVER_CONFIG.trajectory_topic,
            10
        )
        
        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            SERVER_CONFIG.joint_states_topic,
            self._joint_state_callback,
            10
        )
        
        # Only subscribe to speed_scaling for real hardware
        if not SERVER_CONFIG.use_mock_hardware:
            self.speed_scaling_sub = self.create_subscription(
                Float64,
                SERVER_CONFIG.speed_scaling_topic,
                self._speed_scaling_callback,
                10
            )
            self.get_logger().info('Subscribed to speed_scaling (real hardware mode)')
        else:
            # Mock hardware: assume speed_scaling = 1.0 and ready
            self.state_manager.update_speed_scaling(1.0)
            self.get_logger().info('Mock hardware mode: speed_scaling set to 1.0')
        
        self.get_logger().info(f'ROS Bridge initialized')
        self.get_logger().info(f'  Trajectory topic: {SERVER_CONFIG.trajectory_topic}')
        self.get_logger().info(f'  Mock hardware: {SERVER_CONFIG.use_mock_hardware}')
    
    def _joint_state_callback(self, msg: JointState):
        """Handle incoming joint state messages."""
        positions = [0.0] * 6
        velocities = [0.0] * 6
        efforts = [0.0] * 6
        
        name_to_idx = {name: i for i, name in enumerate(self.joint_names)}
        
        for i, name in enumerate(msg.name):
            if name in name_to_idx:
                idx = name_to_idx[name]
                if i < len(msg.position):
                    positions[idx] = msg.position[i]
                if i < len(msg.velocity):
                    velocities[idx] = msg.velocity[i]
                if i < len(msg.effort):
                    efforts[idx] = msg.effort[i]
        
        self.state_manager.update_joint_state(positions, velocities, efforts)
        
        # First joint state received - log it
        if not self._received_joint_state:
            self._received_joint_state = True
            self.get_logger().info(f'Receiving joint states: {[f"{p:.2f}" for p in positions]}')
    
    def _speed_scaling_callback(self, msg: Float64):
        """Handle speed scaling updates (real hardware only)."""
        self.state_manager.update_speed_scaling(msg.data)
        
        if msg.data == 0.0:
            self.get_logger().warn('Speed scaling is 0 - robot will not move')
    
    def send_trajectory(self, positions: list, duration: float) -> bool:
        """Send a trajectory command to the robot."""
        if len(positions) != 6:
            self.get_logger().error(f'Invalid positions length: {len(positions)}')
            return False
        
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9)
        )
        msg.points.append(point)
        
        self.trajectory_pub.publish(msg)
        self.get_logger().info(f'Sent trajectory: {[f"{p:.3f}" for p in positions]}')
        return True
    
    def stop(self) -> bool:
        """Stop robot by commanding current position."""
        current = self.state_manager.get_positions()
        if not current:
            return False
        return self.send_trajectory(current, 0.1)


class ROSManager:
    """Manages ROS2 lifecycle in background thread."""
    
    def __init__(self, state_manager: StateManager):
        self.state_manager = state_manager
        self.bridge: ROSBridge = None
        self._executor = None
        self._thread = None
        self._running = False
    
    def start(self):
        """Start ROS2 in background thread."""
        if self._running:
            return
        
        rclpy.init()
        self.bridge = ROSBridge(self.state_manager)
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self.bridge)
        
        self._running = True
        self._thread = Thread(target=self._spin, daemon=True)
        self._thread.start()
        
        # Wait for first joint state
        timeout = 5.0
        start = time.time()
        while time.time() - start < timeout:
            if self.state_manager.get_state().is_connected:
                break
            time.sleep(0.1)
        
        if not self.state_manager.get_state().is_connected:
            print("Warning: No joint states received within timeout")
    
    def _spin(self):
        """Background ROS2 spin loop."""
        while self._running:
            self._executor.spin_once(timeout_sec=0.05)
    
    def stop(self):
        """Shutdown ROS2."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self.bridge:
            self.bridge.destroy_node()
        rclpy.shutdown()