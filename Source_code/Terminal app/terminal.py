import tkinter as tk
from tkinter import scrolledtext, messagebox
import serial
import threading
import queue
import sys

# --- THEME COLORS ---
BG_COLOR = "#0A0A0A"      # Deep black/dark gray background
FG_COLOR = "#E0E0E0"      # Light gray text for readability
CURSOR_COLOR = "#00FF00"  # Bright green blinking block cursor

class DualSerialMonitor:
    def __init__(self, root, port='COM8', baudrate=115200):
        self.root = root
        
        # --- Main Window (Normal Terminal) ---
        self.root.title(f"Normal Terminal ({port} @ {baudrate})")
        self.root.geometry("600x400+100+100") 
        self.root.configure(bg=BG_COLOR)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # --- Secondary Window (S: Prefixed Terminal) ---
        self.special_window = tk.Toplevel(self.root)
        self.special_window.title("S: Prefixed Terminal")
        self.special_window.geometry("600x400+750+100") 
        self.special_window.configure(bg=BG_COLOR)
        self.special_window.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.data_queue = queue.Queue()
        self.running = True
        
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=0.1)
        except serial.SerialException as e:
            messagebox.showerror("Serial Error", f"Could not open {port}.\n\nError: {e}")
            sys.exit()

        self.build_gui()

        self.read_thread = threading.Thread(target=self.read_from_serial, daemon=True)
        self.read_thread.start()

        self.root.after(100, self.process_queue)

    def build_gui(self):
        # Normal Text Widget
        self.normal_text = scrolledtext.ScrolledText(
            self.root, 
            wrap=tk.WORD, 
            bg=BG_COLOR,
            fg=FG_COLOR,
            blockcursor=True,          
            insertbackground=CURSOR_COLOR,  
            insertwidth=8,
            borderwidth=0,           # Removes standard Windows 3D border
            highlightthickness=0     # Removes focus ring for a cleaner dark mode
        )
        self.normal_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.normal_text.bind("<KeyPress>", self.send_normal_char)
        self.normal_text.focus_set()

        # Special Text Widget
        self.special_text = scrolledtext.ScrolledText(
            self.special_window, 
            wrap=tk.WORD, 
            bg=BG_COLOR,
            fg=FG_COLOR,
            blockcursor=True, 
            insertbackground=CURSOR_COLOR,
            insertwidth=8,
            borderwidth=0,
            highlightthickness=0
        )
        self.special_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.special_text.bind("<KeyPress>", self.send_special_char)

    def read_from_serial(self):
        while self.running:
            try:
                if self.serial_port.in_waiting:
                    raw_data = self.serial_port.readline()
                    try:
                        decoded_text = raw_data.decode('utf-8').strip()
                        if decoded_text:
                            self.data_queue.put(decoded_text)
                    except UnicodeDecodeError:
                        pass
            except Exception as e:
                print(f"Serial read error: {e}")
                break

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
        try:
            self.serial_port.write(message.encode('utf-8'))
        except Exception as e:
            print(f"Failed to send data: {e}")

    def on_closing(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.root.destroy()

if __name__ == "__main__":
    COM_PORT = 'COM8'
    BAUD_RATE = 115200

    root = tk.Tk()
    app = DualSerialMonitor(root, port=COM_PORT, baudrate=BAUD_RATE)
    root.mainloop()
