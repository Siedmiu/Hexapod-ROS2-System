#!/usr/bin/env python3
import asyncio
import subprocess
import websockets
import re

allowed_commands = {
    "bigate11": "ros2 run hex_gz bi_gate.py",
    "ripplegate11": "ros2 run hex_gz ripple_gate.py",
    "trigate11": "ros2 run hex_gz leg_sequence_player.py",
    "wavegate11": "ros2 run hex_gz wave_gate.py",
    "left": "ros2 run hex_gz rotation.py -- --angle ",
    "right": "ros2 run hex_gz rotation.py -- --angle -",
}

async def execute_command(command):
    try:
        subprocess.Popen(command, shell=True, start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
        print(f"Error launching command: {e}")
    return ""

async def handler(websocket, path):
    try:
        async for message in websocket:
            print(f"Received message: {message}")
            if not message.strip():
                continue
            parts = message.strip().split()
            if not parts:
                continue

            cmd_key = parts[0]
            if cmd_key not in allowed_commands:
                continue

            angle = ""
            # tylko left/right mogą mieć kąt
            if cmd_key in ("left", "right"):
                if len(parts) > 1:
                    raw = parts[1]
                    m = re.match(r'[+-]?\d+(\.\d+)?', raw)
                    if m:
                        angle = m.group(0)
                    else:
                        continue

            base = allowed_commands[cmd_key]
            cmd = f"{base}{angle}" if angle else base
            print(f"Executing command: {cmd}")
            await execute_command(cmd)
    except websockets.exceptions.ConnectionClosed:
        pass

async def _main():
    async with websockets.serve(handler, "0.0.0.0", 8765):
        await asyncio.Future()  # run forever

def main():
    asyncio.run(_main())

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass