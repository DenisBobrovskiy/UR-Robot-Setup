#!/usr/bin/env python3
"""Robot Dance Demo - Infinite dancing robot!"""
import asyncio
import random
from client_core import URRobotClient


async def main():
    async with URRobotClient("localhost", 8765) as robot:
        print("Connecting...")
        if not await robot.wait_for_state(timeout=3.0):
            print("Failed to get robot state!")
            return
        
        print(f"Ping: {await robot.ping():.1f} ms")
        print(f"Current positions: {robot.positions}")
        
        # Move to home first
        print("\nMoving to home position...")
        await robot.move_to_named("home", duration=2.0)
        await asyncio.sleep(2.5)
        
        print("\n🎵 Dancing forever! Press Ctrl+C to stop 🎵\n")
        
        # Dance parameters
        wobble = 0.2       # ~6 degrees
        small = 0.05       # ~3 degrees
        speed = 0.4        # seconds per move
        
        loop_count = 0
        
        try:
            while True:
                loop_count += 1
                print(f"--- Dance Loop {loop_count} ---")
                
                # Dance move sets to cycle through
                dance_patterns = [
                    # Pattern 1: Sway
                    [
                        ([wobble, 0, 0, 0, 0, 0], speed),
                        ([-wobble * 2, 0, 0, 0, 0, 0], speed),
                        ([wobble, 0, 0, 0, 0, 0], speed),
                    ],
                    # Pattern 2: Bob
                    [
                        ([0, wobble, 0, 0, 0, 0], speed),
                        ([0, -wobble * 2, 0, 0, 0, 0], speed),
                        ([0, wobble, 0, 0, 0, 0], speed),
                    ],
                    # Pattern 3: Elbow pump
                    [
                        ([0, 0, wobble, 0, 0, 0], speed),
                        ([0, 0, -wobble * 2, 0, 0, 0], speed),
                        ([0, 0, wobble, 0, 0, 0], speed),
                    ],
                    # Pattern 4: Wrist twist
                    [
                        ([0, 0, 0, 0, 0, wobble * 2], speed),
                        ([0, 0, 0, 0, 0, -wobble * 4], speed),
                        ([0, 0, 0, 0, 0, wobble * 2], speed),
                    ],
                    # Pattern 5: Combo groove
                    [
                        ([small, small, -small, 0, 0, small], speed),
                        ([-small*2, -small*2, small*2, 0, 0, -small*2], speed),
                        ([small, small, -small, 0, 0, small], speed),
                    ],
                    # Pattern 6: Random wobbles
                    [
                        ([random.uniform(-small, small), 
                          random.uniform(-small, small), 
                          random.uniform(-small, small), 
                          0, 0, 
                          random.uniform(-wobble, wobble)], 0.3)
                        for _ in range(6)
                    ],
                ]
                
                # Execute each pattern
                for pattern in dance_patterns:
                    for delta, duration in pattern:
                        await robot.move_relative(delta, duration=duration)
                        await asyncio.sleep(duration + 0.1)
        
        except KeyboardInterrupt:
            print("\n\n🛑 Stopping dance...")
        
        finally:
            # Always return home when stopped
            print("Returning to home position...")
            await robot.move_to_named("home", duration=2.0)
            await asyncio.sleep(2.5)
            print(f"Completed {loop_count} dance loops!")
            print("Done!")


if __name__ == "__main__":
    asyncio.run(main())