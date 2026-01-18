"""Configuration for the robot control server."""
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class RobotConfig:
    """Robot configuration."""
    ur_type: str = "ur12e"
    
    # Joint names (standard for all UR robots)
    joint_names: list = field(default_factory=lambda: [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    ])
    
    # Joint limits (radians) - conservative defaults
    joint_limits: list = field(default_factory=lambda: [
        (-6.28, 6.28),   # shoulder_pan
        (-6.28, 6.28),   # shoulder_lift
        (-3.14, 3.14),   # elbow
        (-6.28, 6.28),   # wrist_1
        (-6.28, 6.28),   # wrist_2
        (-6.28, 6.28),   # wrist_3
    ])
    
    # Default motion parameters
    default_duration: float = 2.0
    max_velocity: float = 1.0  # rad/s
    max_acceleration: float = 1.4  # rad/s²


@dataclass
class ServerConfig:
    """Server configuration."""
    host: str = "0.0.0.0"
    port: int = 8765
    state_broadcast_rate: float = 10.0
    use_mock_hardware: bool = False
    
    # Use scaled controller for BOTH mock and real
    trajectory_topic_mock: str = "/scaled_joint_trajectory_controller/joint_trajectory"
    trajectory_topic: str = "/scaled_joint_trajectory_controller/joint_trajectory"
    joint_states_topic: str = "/joint_states"
    speed_scaling_topic: str = "/speed_scaling_state_broadcaster/speed_scaling"
    
    # @property
    # def trajectory_topic(self) -> str:
    #     if self.use_mock_hardware:
    #         return self.trajectory_topic_mock
    #     return self.trajectory_topic_real


# Global configs
ROBOT_CONFIG = RobotConfig()
SERVER_CONFIG = ServerConfig()
