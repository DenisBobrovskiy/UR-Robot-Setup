"""Command execution and validation."""
from typing import Dict, Any, Optional, List

from models import CommandType, CommandResult, MoveCommand
from config import ROBOT_CONFIG
from state_manager import StateManager
from ros_bridge import ROSManager


class CommandExecutor:
    """Executes robot commands with validation."""
    
    def __init__(self, state_manager: StateManager, ros_manager: ROSManager):
        self.state_manager = state_manager
        self.ros_manager = ros_manager
        self.speed_factor = 1.0  # User-adjustable speed multiplier
    
    def execute(self, command: Dict[str, Any]) -> CommandResult:
        """Execute a command and return result."""
        cmd_type = command.get("type", "")
        
        try:
            if cmd_type == CommandType.MOVE_JOINTS:
                return self._move_joints(command)
            
            elif cmd_type == CommandType.MOVE_RELATIVE:
                return self._move_relative(command)
            
            elif cmd_type == CommandType.MOVE_TO_NAMED:
                return self._move_to_named(command)
            
            elif cmd_type == CommandType.STOP:
                return self._stop()
            
            elif cmd_type == CommandType.GET_STATE:
                return self._get_state()
            
            elif cmd_type == CommandType.GET_FULL_STATE:
                return self._get_full_state()
            
            elif cmd_type == CommandType.SAVE_POSITION:
                return self._save_position(command)
            
            elif cmd_type == CommandType.DELETE_POSITION:
                return self._delete_position(command)
            
            elif cmd_type == CommandType.GET_NAMED_POSITIONS:
                return self._get_named_positions()
            
            elif cmd_type == CommandType.SET_SPEED_FACTOR:
                return self._set_speed_factor(command)
            
            elif cmd_type == CommandType.PING:
                return CommandResult(
                    success=True,
                    command="ping",
                    data={"timestamp": command.get("timestamp")}
                )
            
            else:
                return CommandResult(
                    success=False,
                    command=cmd_type,
                    message=f"Unknown command type: {cmd_type}"
                )
        
        except Exception as e:
            return CommandResult(
                success=False,
                command=cmd_type,
                message=f"Error executing command: {str(e)}"
            )
    
    def _check_ready(self) -> Optional[CommandResult]:
        """Check if robot is ready to move. Returns error result or None."""
        state = self.state_manager.get_state()
        
        if not state.is_connected:
            return CommandResult(
                success=False,
                command="move",
                message="Robot not connected - no joint states received"
            )
        
        if state.speed_scaling == 0.0:
            return CommandResult(
                success=False,
                command="move",
                message="Speed scaling is 0 - check teach pendant"
            )
        
        return None
    
    def _move_joints(self, command: Dict) -> CommandResult:
        """Move to absolute joint positions."""
        # Check readiness
        error = self._check_ready()
        if error:
            return error
        
        positions = command.get("positions", [])
        duration = command.get("duration", ROBOT_CONFIG.default_duration)
        duration = duration / self.speed_factor  # Apply speed factor
        
        # Validate
        move_cmd = MoveCommand(positions=positions, duration=duration)
        validation_error = move_cmd.validate(ROBOT_CONFIG.joint_limits)
        if validation_error:
            return CommandResult(
                success=False,
                command="move_joints",
                message=validation_error
            )
        
        # Execute
        success = self.ros_manager.bridge.send_trajectory(positions, duration)
        self.state_manager.set_last_command(f"move_joints: {positions}")
        
        return CommandResult(
            success=success,
            command="move_joints",
            message="Command sent" if success else "Failed to send command",
            data={"target": positions, "duration": duration}
        )
    
    def _move_relative(self, command: Dict) -> CommandResult:
        """Move relative to current position."""
        # Check readiness
        error = self._check_ready()
        if error:
            return error
        
        deltas = command.get("deltas", [])
        duration = command.get("duration", ROBOT_CONFIG.default_duration)
        duration = duration / self.speed_factor
        
        if len(deltas) != 6:
            return CommandResult(
                success=False,
                command="move_relative",
                message=f"Expected 6 delta values, got {len(deltas)}"
            )
        
        # Calculate target
        current = self.state_manager.get_positions()
        if not current or len(current) != 6:
            return CommandResult(
                success=False,
                command="move_relative",
                message="Cannot get current position"
            )
        
        target = [c + d for c, d in zip(current, deltas)]
        
        # Validate target
        move_cmd = MoveCommand(positions=target, duration=duration)
        validation_error = move_cmd.validate(ROBOT_CONFIG.joint_limits)
        if validation_error:
            return CommandResult(
                success=False,
                command="move_relative",
                message=f"Target position invalid: {validation_error}"
            )
        
        # Execute
        success = self.ros_manager.bridge.send_trajectory(target, duration)
        self.state_manager.set_last_command(f"move_relative: {deltas}")
        
        return CommandResult(
            success=success,
            command="move_relative",
            message="Command sent" if success else "Failed to send command",
            data={"current": current, "deltas": deltas, "target": target, "duration": duration}
        )
    
    def _move_to_named(self, command: Dict) -> CommandResult:
        """Move to a named position."""
        error = self._check_ready()
        if error:
            return error
        
        name = command.get("name", "")
        duration = command.get("duration", ROBOT_CONFIG.default_duration)
        duration = duration / self.speed_factor
        
        positions = self.state_manager.get_named_position(name)
        if positions is None:
            available = list(self.state_manager.get_named_positions().keys())
            return CommandResult(
                success=False,
                command="move_to_named",
                message=f"Unknown position '{name}'. Available: {available}"
            )
        
        success = self.ros_manager.bridge.send_trajectory(positions, duration)
        self.state_manager.set_last_command(f"move_to_named: {name}")
        
        return CommandResult(
            success=success,
            command="move_to_named",
            message="Command sent" if success else "Failed to send command",
            data={"name": name, "target": positions, "duration": duration}
        )
    
    def _stop(self) -> CommandResult:
        """Emergency stop."""
        success = self.ros_manager.bridge.stop()
        self.state_manager.set_last_command("stop")
        
        return CommandResult(
            success=success,
            command="stop",
            message="Stop command sent" if success else "Failed to stop"
        )
    
    def _get_state(self) -> CommandResult:
        """Get compact state."""
        state = self.state_manager.get_compact_state()
        return CommandResult(
            success=True,
            command="get_state",
            data=state
        )
    
    def _get_full_state(self) -> CommandResult:
        """Get full robot state."""
        state = self.state_manager.get_state()
        return CommandResult(
            success=True,
            command="get_full_state",
            data=state.to_dict()
        )
    
    def _save_position(self, command: Dict) -> CommandResult:
        """Save a named position."""
        name = command.get("name", "")
        positions = command.get("positions")  # None = use current
        
        if not name:
            return CommandResult(
                success=False,
                command="save_position",
                message="Position name is required"
            )
        
        success = self.state_manager.save_position(name, positions)
        saved_positions = self.state_manager.get_named_position(name)
        
        return CommandResult(
            success=success,
            command="save_position",
            message=f"Position '{name}' saved" if success else "Failed to save",
            data={"name": name, "positions": saved_positions}
        )
    
    def _delete_position(self, command: Dict) -> CommandResult:
        """Delete a named position."""
        name = command.get("name", "")
        success = self.state_manager.delete_position(name)
        
        return CommandResult(
            success=success,
            command="delete_position",
            message=f"Position '{name}' deleted" if success else f"Position '{name}' not found"
        )
    
    def _get_named_positions(self) -> CommandResult:
        """Get all named positions."""
        positions = self.state_manager.get_named_positions()
        return CommandResult(
            success=True,
            command="get_named_positions",
            data=positions
        )
    
    def _set_speed_factor(self, command: Dict) -> CommandResult:
        """Set speed multiplier (0.1 - 2.0)."""
        factor = command.get("factor", 1.0)
        factor = max(0.1, min(2.0, factor))
        self.speed_factor = factor
        
        return CommandResult(
            success=True,
            command="set_speed_factor",
            message=f"Speed factor set to {factor}",
            data={"factor": factor}
        )