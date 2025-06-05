#!/usr/bin/env python3
import asyncio
import subprocess
import websockets

allowed_commands = {
    "bigate11": "ros2 run hex_gz bi_gate.py",
    "ripplegate11": "ros2 run hex_gz ripple_gate.py",
    "trigate11": "ros2 run hex_gz leg_sequence_player.py",
    "wavegate11": "ros2 run hex_gz wave_gate.py",
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
            # print(f"Received message: {message}")
            if not message.strip():
                continue
            parts = message.split()
            if not parts:
                continue
            cmd_key = parts[0]  # ignorujemy tekst po spacji
            if cmd_key in allowed_commands:
                # print(f"Executing command for {cmd_key}: {allowed_commands[cmd_key]}")
                await execute_command(allowed_commands[cmd_key])
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