import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports

class SerialGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("UART Command Sender")

        # Port i prędkość transmisji
        self.port_var = tk.StringVar()
        self.baud_var = tk.IntVar(value=115200)
        self.command_var = tk.StringVar()

        # Serial
        self.ser = None

        # --- GUI layout ---
        frame = ttk.Frame(root, padding=10)
        frame.grid(row=0, column=0)

        ttk.Label(frame, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, values=self.get_serial_ports(), width=15)
        self.port_combo.grid(row=0, column=1, sticky="w")

        ttk.Label(frame, text="Baudrate:").grid(row=1, column=0, sticky="w")
        self.baud_entry = ttk.Entry(frame, textvariable=self.baud_var, width=10)
        self.baud_entry.grid(row=1, column=1, sticky="w")

        self.connect_btn = ttk.Button(frame, text="Connect", command=self.connect)
        self.connect_btn.grid(row=0, column=2, padx=5)

        ttk.Label(frame, text="Command:").grid(row=2, column=0, sticky="w")
        self.command_entry = ttk.Entry(frame, textvariable=self.command_var, width=40)
        self.command_entry.grid(row=2, column=1, columnspan=2, sticky="w")

        self.send_btn = ttk.Button(frame, text="Send", command=self.send_command, state="disabled")
        self.send_btn.grid(row=3, column=1, pady=10, sticky="w")

        self.status_label = ttk.Label(frame, text="Not connected", foreground="red")
        self.status_label.grid(row=4, column=0, columnspan=3, sticky="w")

    def get_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def connect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.connect_btn.config(text="Connect")
            self.send_btn.config(state="disabled")
            self.status_label.config(text="Disconnected", foreground="red")
            return

        port = self.port_var.get()
        baud = self.baud_var.get()
        if not port:
            messagebox.showerror("Error", "Select a serial port")
            return

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=1)
            self.connect_btn.config(text="Disconnect")
            self.send_btn.config(state="normal")
            self.status_label.config(text=f"Connected to {port} @ {baud}bps", foreground="green")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to open serial port:\n{e}")
            self.status_label.config(text="Connection failed", foreground="red")

    def send_command(self):
        if self.ser and self.ser.is_open:
            cmd = self.command_var.get().strip()
            if not cmd:
                messagebox.showwarning("Warning", "Enter a command to send")
                return
            try:
                self.ser.write((cmd + '\n').encode('utf-8'))
                self.status_label.config(text=f"Sent: {cmd}", foreground="blue")
                self.command_var.set("")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to send command:\n{e}")
        else:
            messagebox.showerror("Error", "Not connected to any serial port")

if __name__ == "__main__":
    root = tk.Tk()
    app = SerialGUI(root)
    root.mainloop()