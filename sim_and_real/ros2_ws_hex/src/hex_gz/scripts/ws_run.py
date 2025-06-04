import asyncio
import subprocess
import websockets

allowed_commands = {
    "bigate11": "gnome-calculator",
    "ripplegate11": "gnome-text-editor",
    "trigate11": "gnome-clocks",
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

async def main():
    async with websockets.serve(handler, "0.0.0.0", 8765):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass