#!/usr/bin/env python3
import asyncio
import subprocess
import websockets
import re
import os
import signal

allowed_commands = {
    "bigateForw": "ros2 run hex_gz bi_gate.py",
    "ripplegateForw": "ros2 run hex_gz ripple_gate.py",
    "trigateForw": "ros2 run hex_gz tri_gate.py",
    "wavegateForw": "ros2 run hex_gz wave_gate_square.py",
    "left": "ros2 run hex_gz rotation.py -- --angle ",
    "right": "ros2 run hex_gz rotation.py -- --angle -",
    "bigateBack": "ros2 run hex_gz bi_gate.py -- --back",
    "ripplegateBack": "ros2 run hex_gz ripple_gate.py -- --back",
    "trigateBack": "ros2 run hex_gz tri_gate.py -- --back",
    "wavegateBack": "ros2 run hex_gz wave_gate_square.py -- --back",
    "hi": "ros2 run hex_gz hi.py",
} # + points handled separately

# global list of points for point_to_point
point_list = []
# global list of running processes
running_processes = []

async def execute_command(command):
    try:
        p = subprocess.Popen(command, shell=True, start_new_session=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        running_processes.append(p)
        print(f"Started command: {command}")
        # start process exit monitor
        asyncio.create_task(_process_watcher(p))
    except Exception as e:
        print(f"Error launching command: {e}")
    return ""

async def _process_watcher(p):
    # wait until process finishes
    await asyncio.get_running_loop().run_in_executor(None, p.wait)
    try:
        running_processes.remove(p)
    except ValueError:
        pass
    # do not start anything else after process ends

def stop_all_commands():
    for p in running_processes:
        try:
            os.killpg(p.pid, signal.SIGINT)
        except Exception:
            pass
    running_processes.clear()

async def handler(websocket, path):
    try:
        async for message in websocket:
            print(f"Received WS: {message}")
            msg = message.strip()
            if not msg:
                continue
            parts = msg.split()

            # stop — immediately interrupt all
            if parts[0] == "stop":
                stop_all_commands()
                continue

            # point add <x> <y>
            if parts[0] == "point" and len(parts) >= 4 and parts[1] == "add":
                try:
                    x = float(parts[2]); y = float(parts[3])
                except ValueError:
                    continue
                point_list.append((parts[2], parts[3]))
                continue

            # point execute
            if parts[0] == "point" and parts[1] == "execute":
                if not point_list:
                    continue
                if running_processes:
                    # ignore because a process is still running
                    continue
                args = " ".join(f"{x} {y}" for x, y in point_list)
                cmd = f"ros2 run hex_gz point_to_point.py -- {args}"
                await execute_command(cmd)
                point_list.clear()
                continue

            # other commands
            cmd_key = parts[0]
            if cmd_key in allowed_commands:
                base = allowed_commands[cmd_key]
                angle = ""
                if cmd_key in ("left", "right") and len(parts) > 1:
                    m = re.match(r'[+-]?\d+(\.\d+)?', parts[1])
                    if m:
                        angle = m.group(0)
                    else:
                        continue
                cmd = f"{base}{angle}" if angle else base
                if running_processes:
                    # ignore because a process is still running
                    continue
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