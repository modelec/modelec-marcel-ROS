import serial
import threading
import time
import curses

SERIAL_PORT = '/dev/USB_ODO'
BAUDRATE = 115200
stop_thread = False

def read_serial(ser, log_lines, log_lock):
    """Thread de lecture du port série"""
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
                    log_lines.append(f"[Erreur série] {e}")
        time.sleep(0.05)

def curses_main(stdscr):
    global stop_thread
    curses.curs_set(1)
    stdscr.nodelay(True)
    stdscr.timeout(100)

    log_lines = []
    log_lock = threading.Lock()
    cmd_history = []

    with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
        time.sleep(2)
        reader_thread = threading.Thread(target=read_serial, args=(ser, log_lines, log_lock), daemon=True)
        reader_thread.start()

        user_input = ""

        while True:
            stdscr.clear()
            h, w = stdscr.getmaxyx()

            # Split screen horizontally (80% logs, 20% history)
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
            stdscr.addstr(0, history_start_x, "Historique des commandes:")
            for i, cmd in enumerate(reversed(cmd_history[-(log_height - 2):])):
                stdscr.addnstr(i + 1, history_start_x, cmd, w - history_start_x - 1)

            # --- Input line ---
            stdscr.addstr(log_height, 0, "-" * (w - 1))
            stdscr.addstr(log_height + 1, 0, f"Commande >>> {user_input}")
            stdscr.refresh()

            try:
                ch = stdscr.get_wch()
            except curses.error:
                ch = None

            if ch is None:
                continue

            # Handle key input
            if isinstance(ch, str):
                if ch in ("\n", "\r"):
                    cmd = user_input.strip()
                    if cmd.lower() in ("exit", "quit"):
                        stop_thread = True
                        break
                    if cmd:
                        ser.write((cmd + "\n").encode())
                        cmd_history.append(cmd)
                    user_input = ""
                elif ch in ("\b", "\x7f"):
                    user_input = user_input[:-1]
                elif ch.isprintable():
                    user_input += ch
            elif ch == curses.KEY_BACKSPACE:
                user_input = user_input[:-1]
            elif ch == 27:  # ESC
                stop_thread = True
                break

        stop_thread = True
        reader_thread.join(timeout=1)

curses.wrapper(curses_main)
