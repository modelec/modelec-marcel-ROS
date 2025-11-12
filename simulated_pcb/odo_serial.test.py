import serial
import threading
import time
import curses
import argparse
import serial.tools.list_ports

stop_thread = False

def list_serial_ports():
    """List all available serial ports"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
    else:
        print("Available serial ports:")
        for port in ports:
            print(f"  {port.device} - {port.description}")
    exit(0)

def read_serial(ser, log_lines, log_lock):
    """Thread to read serial output"""
    global stop_thread
    while not stop_thread:
        if ser.in_waiting:
            try:
                line = ser.readline().decode(errors='ignore').strip()
                if line:
                    with log_lock:
                        log_lines.append(line)
                        if len(log_lines) > 1000:
                            log_lines.pop(0)
            except Exception as e:
                with log_lock:
                    log_lines.append(f"[Serial Error] {e}")
        time.sleep(0.05)

def curses_main(stdscr, port, baudrate):
    global stop_thread
    curses.curs_set(1)
    stdscr.nodelay(True)
    stdscr.timeout(100)

    log_lines = []
    log_lock = threading.Lock()
    cmd_history = []
    history_index = -1  # For navigating command history

    with serial.Serial(port, baudrate, timeout=1) as ser:
        time.sleep(2)
        reader_thread = threading.Thread(target=read_serial, args=(ser, log_lines, log_lock), daemon=True)
        reader_thread.start()

        user_input = ""

        while True:
            stdscr.clear()
            h, w = stdscr.getmaxyx()

            # Split screen horizontally (70% logs, 30% command history)
            split_x = int(w * 0.7)
            log_height = h - 2

            # --- Left panel: logs ---
            with log_lock:
                visible_logs = log_lines[-log_height:]
            for i, line in enumerate(visible_logs):
                stdscr.addnstr(i, 0, line, split_x - 1)

            # --- Right panel: command history ---
            stdscr.vline(0, split_x, "|", log_height)
            history_start_x = split_x + 2
            stdscr.addstr(0, history_start_x, "Command History:")
            for i, cmd in enumerate(reversed(cmd_history[-(log_height - 2):])):
                stdscr.addnstr(i + 1, history_start_x, cmd, w - history_start_x - 1)

            # --- Input line ---
            stdscr.addstr(log_height, 0, "-" * (w - 1))
            stdscr.addstr(log_height + 1, 0, f"Command >>> {user_input}")
            stdscr.refresh()

            try:
                ch = stdscr.get_wch()
            except curses.error:
                ch = None

            if ch is None:
                continue

            if isinstance(ch, str):
                if ch in ("\n", "\r"):  # Enter key
                    cmd = user_input.strip()
                    if cmd.lower() in ("exit", "quit"):
                        stop_thread = True
                        break
                    if cmd:
                        ser.write((cmd + "\n").encode())
                        cmd_history.append(cmd)
                        history_index = -1
                    user_input = ""
                elif ch in ("\b", "\x7f"):
                    user_input = user_input[:-1]
                elif ch.isprintable():
                    user_input += ch
            elif ch == curses.KEY_BACKSPACE:
                user_input = user_input[:-1]
            elif ch == curses.KEY_UP:
                if cmd_history:
                    if history_index == -1:
                        history_index = len(cmd_history) - 1
                    elif history_index > 0:
                        history_index -= 1
                    user_input = cmd_history[history_index]
            elif ch == curses.KEY_DOWN:
                if cmd_history:
                    if history_index != -1:
                        history_index += 1
                        if history_index >= len(cmd_history):
                            history_index = -1
                            user_input = ""
                        else:
                            user_input = cmd_history[history_index]
            elif ch == 27:  # ESC
                stop_thread = True
                break

        stop_thread = True
        reader_thread.join(timeout=1)


def main():
    parser = argparse.ArgumentParser(description="Serial monitor with command history and curses UI.")
    parser.add_argument("--port", default="/dev/USB_ODO", help="Serial port to use (default: /dev/USB_ODO)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--list", action="store_true", help="List available serial ports and exit")
    args = parser.parse_args()

    if args.list:
        list_serial_ports()

    curses.wrapper(curses_main, args.port, args.baudrate)


if __name__ == "__main__":
    main()
