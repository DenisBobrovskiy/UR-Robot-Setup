"""Manages robot state and named positions."""
from threading import Lock
from typing import Dict, List, Optional
import time
import json
import os

from models import JointState, RobotState
from config import ROBOT_CONFIG


class StateManager:
    """Thread-safe robot state manager."""
    
    def __init__(self):
        self._lock = Lock()
        self._state = RobotState()
        self._named_positions: Dict[str, List[float]] = {}
        self._positions_file = "/tmp/named_positions.json"
        
        # Load default positions
        self._init_default_positions()
        self._load_positions()
    
    def _init_default_positions(self):
        """Initialize default named positions."""
        self._named_positions = {
            "home": [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
            "up": [0.0, -1.57, 0.0, -1.57, -1.57, 0.0],
            "forward": [0.0, -0.785, 1.57, -2.355, -1.57, 0.0],
            "left": [1.57, -1.57, 1.57, -1.57, -1.57, 0.0],
            "right": [-1.57, -1.57, 1.57, -1.57, -1.57, 0.0],
            "zero": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
    
    def _load_positions(self):
        """Load saved positions from file."""
        try:
            if os.path.exists(self._positions_file):
                with open(self._positions_file, 'r') as f:
                    saved = json.load(f)
                    self._named_positions.update(saved)
        except Exception:
            pass
    
    def _save_positions(self):
        """Save positions to file."""
        try:
            with open(self._positions_file, 'w') as f:
                json.dump(self._named_positions, f)
        except Exception:
            pass
    
    # --- Joint State Updates ---
    
    # def update_joint_state(self, positions: List[float], velocities: List[float], efforts: List[float]):
    #     """Update current joint state (called from ROS subscriber)."""
    #     with self._lock:
    #         self._state.joint_state = JointState(
    #             positions=list(positions),
    #             velocities=list(velocities),
    #             efforts=list(efforts),
    #             timestamp=time.time()
    #         )
    #         self._state.is_connected = True
            
    #         # Check if robot is moving (any velocity above threshold)
    #         self._state.is_moving = any(abs(v) > 0.001 for v in velocities)

    def update_joint_state(self, positions: List[float], velocities: List[float], efforts: List[float]):
        """Update current joint state (called from ROS subscriber)."""
        with self._lock:
            self._state.joint_state = JointState(
                positions=list(positions),
                velocities=list(velocities),
                efforts=list(efforts),
                timestamp=time.time()
            )
            self._state.is_connected = True
            
            # Check if robot is moving
            self._state.is_moving = any(abs(v) > 0.001 for v in velocities)
            
            # For mock hardware: if we're getting joint states, assume ready
            # (speed_scaling topic doesn't exist in simulation)
            if not self._state.is_ready and self._state.speed_scaling == 1.0:
                self._state.is_ready = True
    
    def update_speed_scaling(self, value: float):
        """Update speed scaling factor."""
        with self._lock:
            self._state.speed_scaling = value
            self._state.is_ready = value > 0.0
    
    def update_robot_mode(self, mode: str):
        """Update robot mode."""
        with self._lock:
            self._state.robot_mode = mode
    
    def update_safety_mode(self, mode: str):
        """Update safety mode."""
        with self._lock:
            self._state.safety_mode = mode
    
    def set_last_command(self, command: str):
        """Record last executed command."""
        with self._lock:
            self._state.last_command = command
            self._state.last_command_time = time.time()
    
    # --- State Getters ---
    
    def get_state(self) -> RobotState:
        """Get complete robot state (thread-safe copy)."""
        with self._lock:
            return RobotState(
                joint_state=JointState(
                    positions=list(self._state.joint_state.positions),
                    velocities=list(self._state.joint_state.velocities),
                    efforts=list(self._state.joint_state.efforts),
                    timestamp=self._state.joint_state.timestamp
                ),
                speed_scaling=self._state.speed_scaling,
                is_moving=self._state.is_moving,
                is_connected=self._state.is_connected,
                is_ready=self._state.is_ready,
                safety_mode=self._state.safety_mode,
                robot_mode=self._state.robot_mode,
                last_command=self._state.last_command,
                last_command_time=self._state.last_command_time,
            )
    
    def get_positions(self) -> List[float]:
        """Get current joint positions."""
        with self._lock:
            return list(self._state.joint_state.positions)
    
    def get_compact_state(self) -> Dict:
        """Get minimal state for frequent broadcasts."""
        with self._lock:
            return {
                "positions": list(self._state.joint_state.positions),
                "velocities": list(self._state.joint_state.velocities),
                "speed_scaling": self._state.speed_scaling,
                "is_moving": self._state.is_moving,
                "is_ready": self._state.is_ready,
                "timestamp": self._state.joint_state.timestamp,
            }
    
    # --- Named Positions ---
    
    def get_named_positions(self) -> Dict[str, List[float]]:
        """Get all named positions."""
        with self._lock:
            return dict(self._named_positions)
    
    def get_named_position(self, name: str) -> Optional[List[float]]:
        """Get a specific named position."""
        with self._lock:
            return self._named_positions.get(name)
    
    def save_position(self, name: str, positions: Optional[List[float]] = None) -> bool:
        """Save current or specified position with a name."""
        with self._lock:
            if positions is None:
                positions = list(self._state.joint_state.positions)
            
            if len(positions) != 6:
                return False
            
            self._named_positions[name] = positions
            self._save_positions()
            return True
    
    def delete_position(self, name: str) -> bool:
        """Delete a named position."""
        with self._lock:
            if name in self._named_positions:
                del self._named_positions[name]
                self._save_positions()
                return True
            return False