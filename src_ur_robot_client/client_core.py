
#!/usr/bin/env python3
"""
Fixed Python client for UR Robot Control Server.
"""
import asyncio
import json
import websockets
from typing import Optional, Dict, List, Callable, Any
from dataclasses import dataclass
import time


@dataclass
class RobotState:
    """Robot state from server."""
    positions: List[float]
    velocities: List[float]
    speed_scaling: float
    is_moving: bool
    is_ready: bool
    timestamp: float


class URRobotClient:
    """
    WebSocket client for controlling UR robot.
    
    Example:
        async with URRobotClient("localhost", 8765) as robot:
            await robot.move_to_named("home")
            await robot.move_relative([0.1, 0, 0, 0, 0, 0])
    """
    
    def __init__(self, host: str = "localhost", port: int = 8765):
        self.uri = f"ws://{host}:{port}/ws"
        self.ws: Optional[websockets.WebSocketClientProtocol] = None
        self._state: Optional[RobotState] = None
        self._state_callback: Optional[Callable[[RobotState], None]] = None
        self._listener_task = None
        self._response_queue: asyncio.Queue = None
        self._connected = False
    
    async def __aenter__(self):
        await self.connect()
        return self
    
    async def __aexit__(self, *args):
        await self.disconnect()
    
    async def connect(self):
        """Connect to the robot server."""
        self._response_queue = asyncio.Queue()
        self.ws = await websockets.connect(self.uri)
        self._connected = True
        self._listener_task = asyncio.create_task(self._listen())
        
        # Wait for initial state
        await asyncio.sleep(0.3)
    
    async def disconnect(self):
        """Disconnect from the robot server."""
        self._connected = False
        if self._listener_task:
            self._listener_task.cancel()
            try:
                await self._listener_task
            except asyncio.CancelledError:
                pass
        if self.ws:
            await self.ws.close()
    
    async def _listen(self):
        """Background listener - routes messages to appropriate handlers."""
        try:
            async for message in self.ws:
                if not self._connected:
                    break
                    
                data = json.loads(message)
                msg_type = data.get("type", "")
                
                # State broadcasts (periodic updates)
                if msg_type == "state":
                    self._handle_state(data["data"])
                
                # Full state (on connect)
                elif msg_type == "full_state":
                    self._handle_full_state(data["data"])
                
                # Command responses go to the queue
                elif msg_type in ("command_result", "error", "pong", "named_positions"):
                    await self._response_queue.put(data)
                
                else:
                    # Unknown message type - might be a response
                    await self._response_queue.put(data)
        
        except websockets.ConnectionClosed:
            self._connected = False
        except asyncio.CancelledError:
            pass
    
    def _handle_state(self, data: Dict):
        """Handle periodic state broadcast."""
        self._state = RobotState(
            positions=data.get("positions", []),
            velocities=data.get("velocities", []),
            speed_scaling=data.get("speed_scaling", 0.0),
            is_moving=data.get("is_moving", False),
            is_ready=data.get("is_ready", False),
            timestamp=data.get("timestamp", 0.0)
        )
        
        if self._state_callback:
            self._state_callback(self._state)
    
    def _handle_full_state(self, data: Dict):
        """Handle full state (sent on connect)."""
        joint_state = data.get("joint_state", {})
        self._state = RobotState(
            positions=joint_state.get("positions", []),
            velocities=joint_state.get("velocities", []),
            speed_scaling=data.get("speed_scaling", 0.0),
            is_moving=data.get("is_moving", False),
            is_ready=data.get("is_ready", False),
            timestamp=joint_state.get("timestamp", 0.0)
        )
        
        if self._state_callback:
            self._state_callback(self._state)
    
    async def _send_command(self, command: Dict, timeout: float = 10.0) -> Dict:
        """Send command and wait for response."""
        # Clear any old responses
        while not self._response_queue.empty():
            try:
                self._response_queue.get_nowait()
            except asyncio.QueueEmpty:
                break
        
        # Send command
        await self.ws.send(json.dumps(command))
        
        # Wait for response
        try:
            response = await asyncio.wait_for(
                self._response_queue.get(), 
                timeout=timeout
            )
            return response
        except asyncio.TimeoutError:
            return {"type": "error", "error": "Command timeout"}
    
    def on_state_update(self, callback: Callable[[RobotState], None]):
        """Register callback for state updates."""
        self._state_callback = callback
    
    @property
    def state(self) -> Optional[RobotState]:
        """Get current robot state."""
        return self._state
    
    @property
    def positions(self) -> Optional[List[float]]:
        """Get current joint positions."""
        return self._state.positions if self._state else None
    
    @property
    def is_ready(self) -> bool:
        """Check if robot is ready to move."""
        return self._state.is_ready if self._state else False
    
    @property
    def is_moving(self) -> bool:
        """Check if robot is currently moving."""
        return self._state.is_moving if self._state else False
    
    # --- Movement Commands ---
    
    async def move_joints(self, positions: List[float], duration: float = 2.0) -> Dict:
        """Move to absolute joint positions (radians)."""
        return await self._send_command({
            "type": "move_joints",
            "positions": positions,
            "duration": duration
        })
    
    async def move_relative(self, deltas: List[float], duration: float = 2.0) -> Dict:
        """Move relative to current position."""
        return await self._send_command({
            "type": "move_relative",
            "deltas": deltas,
            "duration": duration
        })
    
    async def move_to_named(self, name: str, duration: float = 2.0) -> Dict:
        """Move to a named position."""
        return await self._send_command({
            "type": "move_to_named",
            "name": name,
            "duration": duration
        })
    
    async def stop(self) -> Dict:
        """Emergency stop."""
        return await self._send_command({"type": "stop"})
    
    # --- State Commands ---
    
    async def get_state(self) -> Dict:
        """Get current robot state."""
        return await self._send_command({"type": "get_state"})
    
    async def get_full_state(self) -> Dict:
        """Get full robot state."""
        return await self._send_command({"type": "get_full_state"})
    
    # --- Position Management ---
    
    async def save_position(self, name: str, positions: Optional[List[float]] = None) -> Dict:
        """Save current or specified position."""
        cmd = {"type": "save_position", "name": name}
        if positions:
            cmd["positions"] = positions
        return await self._send_command(cmd)
    
    async def delete_position(self, name: str) -> Dict:
        """Delete a named position."""
        return await self._send_command({"type": "delete_position", "name": name})
    
    async def get_named_positions(self) -> Dict:
        """Get all named positions."""
        return await self._send_command({"type": "get_named_positions"})
    
    # --- Settings ---
    
    async def set_speed_factor(self, factor: float) -> Dict:
        """Set speed multiplier (0.1 - 2.0)."""
        return await self._send_command({"type": "set_speed_factor", "factor": factor})
    
    async def ping(self) -> float:
        """Ping server, return round-trip time in ms."""
        start = time.time()
        await self._send_command({"type": "ping", "timestamp": start})
        return (time.time() - start) * 1000
    
    # --- Convenience Methods ---
    
    async def wait_until_stopped(self, timeout: float = 30.0) -> bool:
        """Wait until robot stops moving."""
        start = time.time()
        while time.time() - start < timeout:
            if self._state and not self._state.is_moving:
                return True
            await asyncio.sleep(0.1)
        return False
    
    async def wait_for_state(self, timeout: float = 5.0) -> bool:
        """Wait until we have valid state."""
        start = time.time()
        while time.time() - start < timeout:
            if self._state and self._state.positions:
                return True
            await asyncio.sleep(0.1)
        return False