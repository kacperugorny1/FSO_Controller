import tkinter as tk
from tkinter import scrolledtext, messagebox, filedialog
import serial
import threading
import queue
import sys
import time
from datetime import datetime

# --- CONFIGURATION ---
DEBUG_MODE = True  # <--- Prints raw background data to the console

# --- THEME COLORS ---
BG_COLOR = "#0A0A0A"
FG_COLOR = "#E0E0E0"
CURSOR_COLOR = "#00FF00"

class DualSerialMonitor:
    def __init__(self, root, target_port='COM10', baudrate=115200):
        self.root = root
        self.target_port = target_port
        self.baudrate = baudrate
        self.serial_port = None
        self.running = True
        self.data_queue = queue.Queue()
        self.read_thread = None
        
        # --- Main Window (Master Terminal) ---
        self.root.geometry("600x400+100+100") 
        self.root.configure(bg=BG_COLOR)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # --- Secondary Window (Slave Terminal) ---
        self.special_window = tk.Toplevel(self.root)
        self.special_window.geometry("600x400+750+100") 
        self.special_window.configure(bg=BG_COLOR)
        self.special_window.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.build_gui()
        self.update_titles(f"Looking for {self.target_port}...")

        self.log_debug("Application started. Waiting for connection loop...")

        # Start GUI background loops
        self.root.after(100, self.process_queue)
        self.root.after(1000, self.connection_loop)

    def log_debug(self, msg):
        if DEBUG_MODE:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"[{timestamp}] {msg}")

    def build_gui(self):
        self.normal_text = scrolledtext.ScrolledText(
            self.root, wrap=tk.WORD, bg=BG_COLOR, fg=FG_COLOR,
            blockcursor=True, insertbackground=CURSOR_COLOR, insertwidth=8,
            borderwidth=0, highlightthickness=0     
        )
        self.normal_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.normal_text.bind("<KeyPress>", self.send_normal_char)
        self.normal_text.focus_set()

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

    def reset_connection(self):
        """Forces Windows to flush ghost buffers and reset the USB driver state."""
        try:
            self.log_debug("Attempting DTR/RTS hardware reset...")
            self.serial_port.dtr = False
            self.serial_port.rts = False
            time.sleep(0.1)
            self.serial_port.dtr = True
            self.serial_port.rts = True
            time.sleep(0.1)
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            self.log_debug("Hardware reset successful.")
        except Exception as e:
            self.log_debug(f"Hardware reset failed (this is usually okay): {e}")

    def connection_loop(self):
        if not self.running: 
            return

        if self.serial_port is None or not self.serial_port.is_open:
            try:
                self.serial_port = serial.Serial(self.target_port, self.baudrate, timeout=0.05)
                
                # Immediately try to bust any ghost ports created by Windows
                self.reset_connection()
                
                self.log_debug(f"SUCCESS! Connected to {self.target_port}.")
                self.update_titles(f"Connected to {self.target_port} @ {self.baudrate}")
                self.inject_system_message(f"--- CONNECTED TO {self.target_port} ---")
                
                self.read_thread = threading.Thread(target=self.read_from_serial, daemon=True)
                self.read_thread.start()
            except serial.SerialException:
                # Normal behavior when device is simply unplugged
                self.update_titles(f"Waiting for {self.target_port} to be plugged in...")
            
        self.root.after(1000, self.connection_loop)

    def read_from_serial(self):
        self.log_debug("Read thread started successfully.")
        
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                raw_data = self.serial_port.readline()
                
                if raw_data:
                    self.log_debug(f"RAW RX BYTES: {raw_data}")
                    
                    decoded_text = raw_data.decode('utf-8', errors='replace').strip()
                    
                    if decoded_text:
                        self.log_debug(f"DECODED TXT: {decoded_text}")
                        self.data_queue.put(decoded_text)

            except serial.SerialException as e:
                self.log_debug(f"HARDWARE DISCONNECT (SerialException): {e}")
                break 
            except OSError as e:
                self.log_debug(f"HARDWARE DISCONNECT (OSError): {e}")
                break

        self.log_debug("Read thread terminating. Cleaning up port...")
        if self.running and self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
            
            self.root.after(0, lambda: self.update_titles(f"Waiting for {self.target_port} to be plugged in..."))
            self.inject_system_message("--- USB DEVICE DISCONNECTED ---")

    def process_queue(self):
        while not self.data_queue.empty():
            message = self.data_queue.get()
            if message.startswith("S:"):
                self.append_text(self.special_text, message)
            else:
                self.append_text(self.normal_text, message)

        if self.running:
            self.root.after(50, self.process_queue) 

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
                self.log_debug(f"SENDING DATA: {message.encode('utf-8')}")
                self.serial_port.write(message.encode('utf-8'))
            except serial.SerialException as e:
                self.log_debug(f"FAILED TO SEND DATA: {e}")

    def save_log(self, text_widget, default_filename):
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
                self.log_debug(f"Saved file successfully: {file_path}")
            except Exception as e:
                messagebox.showerror("Save Error", f"Failed to save {default_filename}:\n{e}")

    def on_closing(self):
        self.log_debug("User closed the window. Shutting down...")
        self.running = False
        
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.save_log(self.normal_text, f"{timestamp}_master.log")
        self.save_log(self.special_text, f"{timestamp}_slave.log")

        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception:
                pass
            
        self.root.destroy()

if __name__ == "__main__":
    TARGET_USB_PORT = 'COM10'
    BAUD_RATE = 115200

    root = tk.Tk()
    app = DualSerialMonitor(root, target_port=TARGET_USB_PORT, baudrate=BAUD_RATE)
    root.mainloop()