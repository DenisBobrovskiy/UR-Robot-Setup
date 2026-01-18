#!/usr/bin/env python3
"""Entry point for the robot control server."""
import argparse
import uvicorn

from config import SERVER_CONFIG


def main():
    parser = argparse.ArgumentParser(description="UR Robot Control Server")
    parser.add_argument("--host", default=SERVER_CONFIG.host, help="Host to bind to")
    parser.add_argument("--port", type=int, default=SERVER_CONFIG.port, help="Port to bind to")
    parser.add_argument("--mock", action="store_true", help="Use mock hardware controller")
    args = parser.parse_args()
    
    # Update config
    SERVER_CONFIG.use_mock_hardware = args.mock
    
    print(f"Starting UR Robot Control Server")
    print(f"  Host: {args.host}")
    print(f"  Port: {args.port}")
    print(f"  Mock hardware: {args.mock}")
    
    uvicorn.run(
        "ws_handler:app",
        host=args.host,
        port=args.port,
        reload=False,
        log_level="info"
    )


if __name__ == "__main__":
    main()