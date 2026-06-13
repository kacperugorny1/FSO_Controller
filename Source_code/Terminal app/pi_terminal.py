import curses
import serial
import serial.tools.list_ports
import threading
import queue
import time
import sys
from datetime import datetime

class TerminalSerialMonitor:
    def __init__(self, stdscr, baudrate=115200):
        self.stdscr = stdscr
        self.baudrate = baudrate
        self.serial_port = None
        self.running = True
        self.data_queue = queue.Queue()
        
        # Logging lists (stored in memory until Ctrl+C)
        self.master_log = []
        self.slave_log = []

        # Modes: True = Master (Raw), False = Slave (S: prefix)
        self.is_master_mode = True 
        
        self.known_ports = set(p.device for p in serial.tools.list_ports.comports())
        
        # Setup Curses UI
        curses.curs_set(0)       # Hide the blinking terminal cursor
        self.stdscr.nodelay(True) # Make getch() non-blocking
        self.stdscr.timeout(50)   # Main loop refreshes every 50ms

        # Colors
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_BLUE)  # Status Bar
        curses.init_pair(2, curses.COLOR_GREEN, -1)                 # Master Text
        curses.init_pair(3, curses.COLOR_CYAN, -1)                  # Slave Text
        curses.init_pair(4, curses.COLOR_YELLOW, -1)                # System Alerts

        # Calculate window sizes
        self.max_y, self.max_x = self.stdscr.getmaxyx()
        half_x = self.max_x // 2

        # Create split windows
        self.win_master = curses.newwin(self.max_y - 2, half_x, 0, 0)
        self.win_slave = curses.newwin(self.max_y - 2, self.max_x - half_x, 0, half_x)
        self.win_status = curses.newwin(2, self.max_x, self.max_y - 2, 0)

        self.win_master.scrollok(True)
        self.win_slave.scrollok(True)

        self.device_status = "Waiting for new device connection..."
        
        # Start background hardware scanner
        self.scan_thread = threading.Thread(target=self.scan_for_device_loop, daemon=True)
        self.scan_thread.start()

    def print_to_win(self, window, text, color_pair):
        """Safely appends text to a scrolling curses window."""
        try:
            window.addstr(text + "\n", curses.color_pair(color_pair))
            window.refresh()
        except curses.error:
            pass # Ignore curses boundary errors when resizing

    def inject_system_message(self, message):
        """Adds a yellow alert to both screens."""
        self.data_queue.put(("SYS", f"--- {message} ---"))

    def scan_for_device_loop(self):
        """Background thread that polls for new USB/Serial devices."""
        while self.running:
            current_ports = set(p.device for p in serial.tools.list_ports.comports())

            if self.serial_port is None or not self.serial_port.is_open:
                new_ports = current_ports - self.known_ports

                if new_ports:
                    target_port = list(new_ports)[0]
                    try:
                        self.serial_port = serial.Serial(target_port, self.baudrate, timeout=0.1)
                        self.device_status = f"Connected to {target_port} @ {self.baudrate}"
                        self.inject_system_message(f"CONNECTED TO {target_port}")
                        
                        # Start reading data from the new device
                        threading.Thread(target=self.read_from_serial, daemon=True).start()
                    except Exception as e:
                        self.device_status = f"Error connecting: {e}"
                
            self.known_ports = current_ports
            time.sleep(1) # Wait 1 second before scanning again

    def read_from_serial(self):
        """Background thread to read incoming serial data."""
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    raw_data = self.serial_port.readline()
                    try:
                        decoded = raw_data.decode('utf-8').strip()
                        if decoded:
                            self.data_queue.put(("DATA", decoded))
                    except UnicodeDecodeError:
                        pass
            except serial.SerialException:
                break 
            except OSError:
                break

        if self.running and self.serial_port:
            self.serial_port.close()
            self.serial_port = None
            self.device_status = "Waiting for new device connection..."
            self.inject_system_message("DEVICE DISCONNECTED")

    def write_serial(self, message):
        """Sends raw data over the serial connection."""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(message.encode('utf-8'))
            except serial.SerialException:
                pass

    def draw_status_bar(self):
        """Updates the bottom 2 lines of the terminal."""
        try:
            self.win_status.erase()
            
            # Line 1: Hardware Status
            status_line = f" STATUS: {self.device_status} ".ljust(self.max_x)
            self.win_status.addstr(0, 0, status_line, curses.color_pair(1) | curses.A_BOLD)
            
            # Line 2: Active Mode
            if self.is_master_mode:
                mode_str = " [ MODE: MASTER ] - Typing sends characters normally. | Press '\\' to switch."
            else:
                mode_str = " [ MODE: SLAVE  ] - Typing instantly prepends 'S:' to characters. | Press '\\' to switch."
            
            mode_line = mode_str.ljust(self.max_x)
            self.win_status.addstr(1, 0, mode_line, curses.color_pair(1))
            
            self.win_status.refresh()
        except curses.error:
            pass

    def save_logs(self):
        """Called automatically when Ctrl+C is pressed."""
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        
        master_file = f"{timestamp}_master.log"
        slave_file = f"{timestamp}_slave.log"
        
        if self.master_log:
            with open(master_file, 'w', encoding='utf-8') as f:
                f.write("\n".join(self.master_log) + "\n")
                
        if self.slave_log:
            with open(slave_file, 'w', encoding='utf-8') as f:
                f.write("\n".join(self.slave_log) + "\n")

    def run(self):
        """Main Curses Event Loop"""
        # Draw initial headers
        self.print_to_win(self.win_master, "--- MASTER TERMINAL ---", 2)
        self.print_to_win(self.win_slave, "--- SLAVE (S:) TERMINAL ---", 3)

        while self.running:
            self.draw_status_bar()

            # Process incoming data queue
            while not self.data_queue.empty():
                msg_type, content = self.data_queue.get()
                
                if msg_type == "SYS":
                    self.print_to_win(self.win_master, content, 4)
                    self.print_to_win(self.win_slave, content, 4)
                elif msg_type == "DATA":
                    if content.startswith("S:"):
                        self.print_to_win(self.win_slave, content, 3)
                        self.slave_log.append(content)
                    else:
                        self.print_to_win(self.win_master, content, 2)
                        self.master_log.append(content)

            # Process Keystrokes
            try:
                c = self.stdscr.getch()
                
                if c == -1:
                    continue # No key pressed

                if c == 3: # ASCII value for Ctrl+C
                    self.running = False
                    break
                    
                if c == ord('\\'):
                    self.is_master_mode = not self.is_master_mode # Toggle mode
                    continue
                
                # Handle Enter Key
                if c in [10, 13, curses.KEY_ENTER]:
                    self.write_serial("\r\n")
                    continue
                
                # Handle standard printable characters (PuTTY style, send on click)
                if 32 <= c <= 126:
                    char = chr(c)
                    if self.is_master_mode:
                        self.write_serial(char)
                    else:
                        self.write_serial(f"S:{char}")

            except KeyboardInterrupt:
                self.running = False
                break

        # Cleanup hardware on exit
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

def main(stdscr):
    app = TerminalSerialMonitor(stdscr, baudrate=115200)
    app.run()
    app.save_logs()

if __name__ == "__main__":
    try:
        # curses.wrapper automatically handles setup and safe teardown of the terminal
        curses.wrapper(main)
        print("Logs successfully saved. Exited cleanly.")
    except KeyboardInterrupt:
        pass
