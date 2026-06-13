import tkinter as tk
from tkinter import scrolledtext, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import queue
import sys
from datetime import datetime

# --- THEME COLORS ---
BG_COLOR = "#0A0A0A"      # Deep black/dark gray background
FG_COLOR = "#E0E0E0"      # Light gray text for readability
CURSOR_COLOR = "#00FF00"  # Bright green blinking block cursor

class DualSerialMonitor:
    def __init__(self, root, baudrate=115200):
        self.root = root
        self.baudrate = baudrate
        self.serial_port = None
        self.running = True
        self.data_queue = queue.Queue()
        self.read_thread = None

        self.known_ports = set(p.device for p in serial.tools.list_ports.comports())
        
        # --- Main Window (Normal Terminal) ---
        self.root.geometry("600x400+100+100") 
        self.root.configure(bg=BG_COLOR)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # --- Secondary Window (S: Prefixed Terminal) ---
        self.special_window = tk.Toplevel(self.root)
        self.special_window.geometry("600x400+750+100") 
        self.special_window.configure(bg=BG_COLOR)
        self.special_window.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.build_gui()
        self.update_titles("Waiting for new device connection...")

        # Start GUI background loops
        self.root.after(100, self.process_queue)
        self.root.after(1000, self.scan_for_device)

    def build_gui(self):
        # Normal Text Widget (Master)
        self.normal_text = scrolledtext.ScrolledText(
            self.root, wrap=tk.WORD, bg=BG_COLOR, fg=FG_COLOR,
            blockcursor=True, insertbackground=CURSOR_COLOR, insertwidth=8,
            borderwidth=0, highlightthickness=0     
        )
        self.normal_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.normal_text.bind("<KeyPress>", self.send_normal_char)
        self.normal_text.focus_set()

        # Special Text Widget (Slave)
        self.special_text = scrolledtext.ScrolledText(
            self.special_window, wrap=tk.WORD, bg=BG_COLOR, fg=FG_COLOR,
            blockcursor=True, insertbackground=CURSOR_COLOR, insertwidth=8,
            borderwidth=0, highlightthickness=0
        )
        self.special_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.special_text.bind("<KeyPress>", self.send_special_char)

    def update_titles(self, status):
        self.root.title(f"Normal Terminal (Master) - {status}")
        self.special_window.title(f"S: Prefixed Terminal (Slave) - {status}")

    def inject_system_message(self, message):
        self.data_queue.put(message)
        self.data_queue.put(f"S:{message}")

    def scan_for_device(self):
        if not self.running: 
            return

        current_ports = set(p.device for p in serial.tools.list_ports.comports())

        if self.serial_port is None or not self.serial_port.is_open:
            new_ports = current_ports - self.known_ports

            if new_ports:
                target_port = list(new_ports)[0]
                try:
                    self.serial_port = serial.Serial(target_port, self.baudrate, timeout=0.1)
                    self.update_titles(f"Connected to {target_port} @ {self.baudrate}")
                    self.inject_system_message(f"--- CONNECTED TO {target_port} ---")
                    
                    self.read_thread = threading.Thread(target=self.read_from_serial, daemon=True)
                    self.read_thread.start()
                except serial.SerialException as e:
                    print(f"Failed to connect to {target_port}: {e}")
            
        self.known_ports = current_ports
        self.root.after(1000, self.scan_for_device)

    def read_from_serial(self):
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    raw_data = self.serial_port.readline()
                    try:
                        decoded_text = raw_data.decode('utf-8').strip()
                        if decoded_text:
                            self.data_queue.put(decoded_text)
                    except UnicodeDecodeError:
                        pass
            except serial.SerialException:
                break 
            except OSError:
                break

        if self.running and self.serial_port:
            self.serial_port.close()
            self.serial_port = None
            self.update_titles("Waiting for new device connection...")
            self.inject_system_message("--- DEVICE DISCONNECTED ---")

    def process_queue(self):
        while not self.data_queue.empty():
            message = self.data_queue.get()
            if message.startswith("S:"):
                self.append_text(self.special_text, message)
            else:
                self.append_text(self.normal_text, message)

        if self.running:
            self.root.after(100, self.process_queue)

    def append_text(self, text_widget, message):
        text_widget.insert(tk.END, message + "\n")
        text_widget.see(tk.END)

    def send_normal_char(self, event):
        self.normal_text.mark_set(tk.INSERT, tk.END)
        self.normal_text.see(tk.END)

        if event.keysym == 'Return':
            self.write_serial('\r\n')
            return "break"
        if event.keysym in ('BackSpace', 'Delete', 'Tab'):
            return "break"
        if event.char and event.char.isprintable():
            self.write_serial(event.char)
            return "break" 
        return None 

    def send_special_char(self, event):
        self.special_text.mark_set(tk.INSERT, tk.END)
        self.special_text.see(tk.END)

        if event.keysym == 'Return':
            self.write_serial('\r\n')
            return "break"
        if event.keysym in ('BackSpace', 'Delete', 'Tab'):
            return "break"
        if event.char and event.char.isprintable():
            self.write_serial(f"S:{event.char}")
            return "break"
        return None

    def write_serial(self, message):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(message.encode('utf-8'))
            except serial.SerialException:
                pass 

    def save_log(self, text_widget, default_filename):
        """Opens a save dialog with a pre-filled filename and saves the widget's text."""
        # Only prompt to save if there is actually text to save
        content = text_widget.get("1.0", tk.END).strip()
        if not content:
            return

        file_path = filedialog.asksaveasfilename(
            initialfile=default_filename,
            title=f"Save Log: {default_filename}", 
            defaultextension=".log",
            filetypes=[("Log Files", "*.log"), ("Text Files", "*.txt"), ("All Files", "*.*")]
        )
        if file_path:
            try:
                with open(file_path, 'w', encoding='utf-8') as file:
                    file.write(content + "\n")
            except Exception as e:
                messagebox.showerror("Save Error", f"Failed to save {default_filename}:\n{e}")

    def on_closing(self):
        """Intercepts window close, prompts for saves, then kills the app."""
        self.running = False
        
        # Generate the timestamp
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        
        # Prompt to save Master log
        self.save_log(self.normal_text, f"{timestamp}_master.log")
        
        # Prompt to save Slave log
        self.save_log(self.special_text, f"{timestamp}_slave.log")

        # Clean up serial connection
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            
        # Destroy the application
        self.root.destroy()

if __name__ == "__main__":
    BAUD_RATE = 115200

    root = tk.Tk()
    app = DualSerialMonitor(root, baudrate=BAUD_RATE)
    root.mainloop()
