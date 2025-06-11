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
} # + point obsłużone osobno

# globalna lista punktów do point_to_point
point_list = []
# globalna lista uruchomionych procesów
running_processes = []
# zmienna przechowująca ostatnie oczekujące polecenie
pending_command = None

async def execute_command(command):
    try:
        p = subprocess.Popen(command, shell=True, start_new_session=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        running_processes.append(p)
        print(f"Started command: {command}")
        # uruchom monitor zakończenia procesu
        asyncio.create_task(_process_watcher(p))
    except Exception as e:
        print(f"Error launching command: {e}")
    return ""

async def _process_watcher(p):
    global pending_command
    # poczekaj aż proces się zakończy
    await asyncio.get_running_loop().run_in_executor(None, p.wait)
    try:
        running_processes.remove(p)
    except ValueError:
        pass
    # jeśli jest oczekujące polecenie, uruchom je
    if pending_command:
        cmd = pending_command
        pending_command = None
        await execute_command(cmd)

def stop_all_commands():
    global pending_command
    for p in running_processes:
        try:
            os.killpg(p.pid, signal.SIGINT)
        except Exception:
            pass
    running_processes.clear()
    # reset oczekującego polecenia
    pending_command = None

async def handler(websocket, path):
    global pending_command
    try:
        async for message in websocket:
            print(f"Received WS: {message}")
            msg = message.strip()
            if not msg:
                continue
            parts = msg.split()

            # stop — natychmiast przerwij wszystko
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
                args = " ".join(f"{x} {y}" for x, y in point_list)
                cmd = f"ros2 run hex_gz point_to_point.py -- {args}"
                # zamiast kolejki: nadpisujemy pending_command
                if running_processes:
                    pending_command = cmd
                else:
                    await execute_command(cmd)
                point_list.clear()
                continue

            # pozostałe komendy
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
                    pending_command = cmd
                else:
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