import serial
import time
import math
import threading
import argparse


class SimulatedPCB:
    def __init__(self, port='/dev/pts/6', baud=115200):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.running = True
        self.start = False

        self.x = 1225.0
        self.y = 300.0
        self.theta = math.pi / 2

        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0

        self.last_update = time.time()
        self.last_send = time.time()

        self.pid = [0, 0, 0]
        self.tof = {0: 1000, 1: 1000, 2: 1000}

        # --- waypoint system ---
        self.waypoints = {}          # id -> waypoint
        self.waypoint_queue = []     # FIFO execution
        self.current_wp_id = None

        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def update_position(self):
        now = time.time()
        dt = now - self.last_update
        self.last_update = now

        if self.start and self.waypoint_queue:
            wp_id = self.waypoint_queue[0]
            wp = self.waypoints[wp_id]

            dx = wp['x'] - self.x
            dy = wp['y'] - self.y
            distance = math.hypot(dx, dy)

            target_angle = math.atan2(dy, dx)
            theta_error = self.normalize_angle(wp['theta'] - self.theta)

            # --- motion ---
            if wp['type'] == 1:  # precise waypoint
                speed = min(120.0, distance * 1)
                self.vtheta = max(-6.0, min(6.0, theta_error * 2))
            else:  # transit waypoint
                speed = min(180.0, distance * 2)
                self.vtheta = 0.0

            self.vx = speed * math.cos(target_angle)
            self.vy = speed * math.sin(target_angle)

            reached_pos = distance < 5.0
            reached_theta = abs(theta_error) < 0.15 or wp['type'] == 0

            if reached_pos and reached_theta:
                self.vx = self.vy = self.vtheta = 0.0
                self.current_wp_id = wp_id

                self.ser.write(f"SET;WAYPOINT;REACH;{wp_id}\n".encode())

                self.waypoint_queue.pop(0)
                del self.waypoints[wp_id]

        else:
            self.vx = self.vy = self.vtheta = 0.0

        # --- integrate ---
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.theta = self.normalize_angle(self.theta + self.vtheta * dt)

        # --- periodic position ---
        if self.start and now - self.last_send > 0.1:
            self.ser.write(
                f"SET;POS;{int(self.x)};{int(self.y)};{self.theta:.5f}\n".encode()
            )
            self.last_send = now

    def run(self):
        while self.running:
            self.update_position()

            if self.ser.in_waiting:
                msg = self.ser.readline().decode('utf-8').strip()
                print(f"[RX] {msg}")
                self.handle_message(msg)

            time.sleep(0.01)

    def handle_message(self, msg):
        if msg == "GET;POS":
            self.ser.write(
                f"SET;POS;{int(self.x)};{int(self.y)};{self.theta:.3f}\n".encode()
            )

        elif msg == "GET;SPEED":
            self.ser.write(
                f"SET;SPEED;{int(self.vx)};{int(self.vy)};{int(self.vtheta)}\n".encode()
            )

        elif msg.startswith("GET;DIST;"):
            n = int(msg.split(';')[2])
            dist = self.tof.get(n, 0)
            self.ser.write(f"SET;DIST;{n};{dist}\n".encode())

        elif msg == "GET;PID":
            self.ser.write(f"SET;PID;{self.pid[0]};{self.pid[1]};{self.pid[2]}\n".encode())

        elif msg.startswith("SET;PID"):
            self.ser.write(b"OK;PID;1\n")

        elif msg.startswith("SET;START;"):
            self.start = msg.split(';')[2] == '1'
            self.ser.write(b"OK;START;1\n")

        # --- MULTI WAYPOINT (clears previous) ---
        elif msg.startswith("SET;WAYPOINT;"):
            parts = msg.split(';')[2:]

            if len(parts) % 5 != 0:
                self.ser.write(b"ERR;WAYPOINT;0\n")
                return

            # clear existing waypoints
            self.waypoints.clear()
            self.waypoint_queue.clear()

            for i in range(0, len(parts), 5):
                wp = {
                    'id': int(parts[i]),
                    'type': int(parts[i + 1]),
                    'x': float(parts[i + 2]),
                    'y': float(parts[i + 3]),
                    'theta': float(parts[i + 4]),
                }

                self.waypoints[wp['id']] = wp
                self.waypoint_queue.append(wp['id'])

                self.ser.write(f"OK;WAYPOINT;{wp['id']}\n".encode())

        elif msg.startswith("SET;POS"):
            parts = msg.split(';')
            self.x = float(parts[2])
            self.y = float(parts[3])
            self.theta = float(parts[4])
            self.ser.write(b"OK;POS;1\n")

    def stop(self):
        self.running = False
        self.thread.join()
        self.ser.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Simulated PCB")
    parser.add_argument('--port', type=str, default='/dev/pts/6')
    args = parser.parse_args()

    sim = None
    try:
        sim = SimulatedPCB(port=args.port)
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        if sim:
            sim.stop()
        print("Simulation stopped")
