"""WebSocket server handler."""
import asyncio
import json
from typing import Set
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager

from config import SERVER_CONFIG
from state_manager import StateManager
from ros_bridge import ROSManager
from command_executor import CommandExecutor
from models import ResponseType


class WebSocketHandler:
    """Handles WebSocket connections and message routing."""
    
    def __init__(self):
        self.state_manager = StateManager()
        self.ros_manager = ROSManager(self.state_manager)
        self.command_executor = CommandExecutor(self.state_manager, self.ros_manager)
        self.clients: Set[WebSocket] = set()
        self._broadcast_task = None
    
    async def start(self):
        """Start the handler."""
        self.ros_manager.start()
        self._broadcast_task = asyncio.create_task(self._broadcast_loop())
    
    async def stop(self):
        """Stop the handler."""
        if self._broadcast_task:
            self._broadcast_task.cancel()
        self.ros_manager.stop()
    
    async def connect(self, websocket: WebSocket):
        """Handle new WebSocket connection."""
        await websocket.accept()
        self.clients.add(websocket)
        
        # Send initial state
        state = self.state_manager.get_state()
        await websocket.send_text(json.dumps({
            "type": ResponseType.FULL_STATE,
            "data": state.to_dict()
        }))
    
    def disconnect(self, websocket: WebSocket):
        """Handle WebSocket disconnection."""
        self.clients.discard(websocket)
    
    async def handle_message(self, websocket: WebSocket, message: str) -> str:
        """Process incoming message and return response."""
        try:
            command = json.loads(message)
        except json.JSONDecodeError:
            return json.dumps({
                "type": ResponseType.ERROR,
                "error": "Invalid JSON"
            })
        
        # Execute command
        result = self.command_executor.execute(command)
        
        return json.dumps({
            "type": ResponseType.COMMAND_RESULT,
            "data": result.to_dict()
        })
    
    async def _broadcast_loop(self):
        """Broadcast state to all clients at configured rate."""
        interval = 1.0 / SERVER_CONFIG.state_broadcast_rate
        
        while True:
            try:
                if self.clients:
                    state = self.state_manager.get_compact_state()
                    message = json.dumps({
                        "type": ResponseType.STATE,
                        "data": state
                    })
                    
                    disconnected = set()
                    for client in self.clients:
                        try:
                            await client.send_text(message)
                        except Exception:
                            disconnected.add(client)
                    
                    self.clients -= disconnected
                
                await asyncio.sleep(interval)
            
            except asyncio.CancelledError:
                break
            except Exception:
                await asyncio.sleep(interval)


# Global handler instance
ws_handler = WebSocketHandler()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifecycle."""
    await ws_handler.start()
    yield
    await ws_handler.stop()


# Create FastAPI app
app = FastAPI(
    title="UR Robot Control API",
    version="1.0.0",
    lifespan=lifespan
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """Main WebSocket endpoint."""
    await ws_handler.connect(websocket)
    
    try:
        while True:
            message = await websocket.receive_text()
            response = await ws_handler.handle_message(websocket, message)
            await websocket.send_text(response)
    except WebSocketDisconnect:
        ws_handler.disconnect(websocket)


@app.get("/health")
async def health():
    """Health check endpoint."""
    state = ws_handler.state_manager.get_state()
    return {
        "status": "ok",
        "connected": state.is_connected,
        "ready": state.is_ready
    }


@app.get("/state")
async def get_state():
    """REST endpoint for current state."""
    return ws_handler.state_manager.get_state().to_dict()


@app.get("/positions")
async def get_positions():
    """REST endpoint for named positions."""
    return ws_handler.state_manager.get_named_positions()