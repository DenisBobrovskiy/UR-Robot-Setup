"""Data models for robot control."""
from dataclasses import dataclass, field, asdict
from typing import Optional, List, Dict, Any
from enum import Enum
import time


class CommandType(str, Enum):
    """Available command types."""
    MOVE_JOINTS = "move_joints"
    MOVE_RELATIVE = "move_relative"
    MOVE_TO_NAMED = "move_to_named"
    STOP = "stop"
    GET_STATE = "get_state"
    GET_FULL_STATE = "get_full_state"
    SAVE_POSITION = "save_position"
    DELETE_POSITION = "delete_position"
    GET_NAMED_POSITIONS = "get_named_positions"
    SET_SPEED_FACTOR = "set_speed_factor"
    PING = "ping"


class ResponseType(str, Enum):
    """Response message types."""
    STATE = "state"
    FULL_STATE = "full_state"
    COMMAND_RESULT = "command_result"
    ERROR = "error"
    PONG = "pong"
    NAMED_POSITIONS = "named_positions"


@dataclass
class JointState:
    """Current joint state."""
    positions: List[float] = field(default_factory=list)
    velocities: List[float] = field(default_factory=list)
    efforts: List[float] = field(default_factory=list)
    timestamp: float = 0.0
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass 
class RobotState:
    """Complete robot state."""
    joint_state: JointState = field(default_factory=JointState)
    speed_scaling: float = 1.0
    is_moving: bool = False
    is_connected: bool = False
    is_ready: bool = False
    safety_mode: str = "NORMAL"
    robot_mode: str = "RUNNING"
    last_command: Optional[str] = None
    last_command_time: float = 0.0
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "joint_state": self.joint_state.to_dict(),
            "speed_scaling": self.speed_scaling,
            "is_moving": self.is_moving,
            "is_connected": self.is_connected,
            "is_ready": self.is_ready,
            "safety_mode": self.safety_mode,
            "robot_mode": self.robot_mode,
            "last_command": self.last_command,
            "last_command_time": self.last_command_time,
        }


@dataclass
class MoveCommand:
    """Movement command."""
    positions: List[float]
    duration: float = 2.0
    is_relative: bool = False
    
    def validate(self, joint_limits: List[tuple]) -> Optional[str]:
        """Validate command. Returns error message or None if valid."""
        if len(self.positions) != 6:
            return f"Expected 6 joint positions, got {len(self.positions)}"
        
        if self.duration <= 0:
            return f"Duration must be positive, got {self.duration}"
        
        if not self.is_relative:
            for i, (pos, (min_val, max_val)) in enumerate(zip(self.positions, joint_limits)):
                if not (min_val <= pos <= max_val):
                    return f"Joint {i} position {pos} out of range [{min_val}, {max_val}]"
        
        return None


@dataclass
class CommandResult:
    """Result of a command execution."""
    success: bool
    command: str
    message: str = ""
    data: Optional[Dict[str, Any]] = None
    timestamp: float = field(default_factory=time.time)
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)