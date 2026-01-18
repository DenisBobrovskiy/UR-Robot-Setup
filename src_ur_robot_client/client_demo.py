"""Example: Control robot from external application."""
import asyncio
from client_core import URRobotClient


async def main():
    async with URRobotClient("localhost", 8765) as robot:
        # Wait for state to be populated
        print("Connecting...")
        if not await robot.wait_for_state(timeout=3.0):
            print("Failed to get robot state!")
            return
        
        # Check connection
        print(f"Ping: {await robot.ping():.1f} ms")
        print(f"Ready: {robot.is_ready}")
        print(f"Current positions: {robot.positions}")
        
        # Move to home
        print("\nMoving to home...")
        result = await robot.move_to_named("home", duration=3.0)
        print(f"Result: {result['data']['success']}")
        
        # Wait for movement to complete
        await asyncio.sleep(3.5)
        
        # Relative movements
        print("\nMoving shoulder left...")
        result = await robot.move_relative([0.2, 0, 0, 0, 0, 0], duration=2.0)
        print(f"Result: {result['data']['success']}")
        await asyncio.sleep(2.5)
        
        print("\nMoving shoulder right...")
        result = await robot.move_relative([-0.4, 0, 0, 0, 0, 0], duration=2.0)
        print(f"Result: {result['data']['success']}")
        await asyncio.sleep(2.5)
        
        # Back to center
        print("\nMoving back to center...")
        result = await robot.move_relative([0.2, 0, 0, 0, 0, 0], duration=2.0)
        print(f"Result: {result['data']['success']}")
        await asyncio.sleep(2.5)
        
        # Show final position
        print(f"\nFinal positions: {robot.positions}")
        print("\nDone!")


if __name__ == "__main__":
    asyncio.run(main())