import serial
import time
import math
import threading

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
        self.waypoints = {}  # waypoints by id
        self.waypoint_order = []  # ordered list of ids
        self.current_wp_id = None

        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def update_position(self):
        now = time.time()
        dt = now - self.last_update
        self.last_update = now

        if self.start:
            if self.waypoint_order:
                wp = self.waypoints[self.waypoint_order[0]]
                dx = wp['x'] - self.x
                dy = wp['y'] - self.y
                distance = math.hypot(dx, dy)
                angle = math.atan2(dy, dx)
                angle_diff = self.normalize_angle(wp['theta'] - self.theta)

                if wp['type'] == 1:
                    if distance < 5.0 and abs(angle_diff) < 0.15:
                        self.vx = 0.0
                        self.vy = 0.0
                        self.vtheta = 0.0
                        self.ser.write(f'SET;WAYPOINT;{wp["id"]}\n'.encode('utf-8'))
                        self.current_wp_id = wp['id']
                        self.waypoint_order.pop(0)
                        del self.waypoints[wp['id']]
                    else:
                        speed = min(300.0, distance * 3)
                        self.vx = speed * math.cos(angle)
                        self.vy = speed * math.sin(angle)
                        self.vtheta = max(-9.0, min(9.0, angle_diff * 2))
                else:
                    speed = min(600.0, distance * 4)
                    self.vx = speed * math.cos(angle)
                    self.vy = speed * math.sin(angle)
                    self.vtheta = 0.0
                    if distance < 100:
                        self.ser.write(f'SET;WAYPOINT;{wp["id"]}\n'.encode('utf-8'))
                        self.current_wp_id = wp['id']
                        self.waypoint_order.pop(0)
                        del self.waypoints[wp['id']]
            else:
                self.vx = 0.0
                self.vy = 0.0
                self.vtheta = 0.0

            self.x += self.vx * dt
            self.y += self.vy * dt
            self.theta += self.vtheta * dt
            self.theta = self.normalize_angle(self.theta)

            if now - self.last_send > 0.1:
                self.ser.write(f'SET;POS;{int(self.x)};{int(self.y)};{self.theta:.5f}\n'.encode())
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
            self.ser.write(f'SET;POS;{int(self.x)};{int(self.y)};{self.theta:.3f}\n'.encode())

        elif msg == "GET;SPEED":
            self.ser.write(f'SET;SPEED;{int(self.vx)};{int(self.vy)};{int(self.vtheta)}\n'.encode())

        elif msg.startswith("GET;DIST;"):
            n = int(msg.split(';')[2])
            dist = self.tof.get(n, 0)
            self.ser.write(f'SET;DIST;{n};{dist}\n'.encode())

        elif msg == "GET;PID":
            self.ser.write(f'SET;PID;{self.pid[0]};{self.pid[1]};{self.pid[2]}\n'.encode())

        elif msg.startswith("SET;PID"):
            self.pid = list(map(int, msg.split(';')[2:5]))
            self.ser.write(f'OK;PID;1\n'.encode())

        elif msg.startswith("SET;START"):
            self.start = msg.split(';')[2] == '1'
            self.ser.write(f'OK;START;1\n'.encode())

        elif msg.startswith("SET;WAYPOINT"):
            parts = msg.split(';')
            if len(parts) >= 7:
                wp = {
                    'id': int(parts[2]),
                    'type': int(parts[3]),
                    'x': float(parts[4]),
                    'y': float(parts[5]),
                    'theta': float(parts[6])
                }
                self.waypoints[wp['id']] = wp
                if wp['id'] not in self.waypoint_order:
                    self.waypoint_order.append(wp['id'])
                self.ser.write(f'OK;WAYPOINT;{wp["id"]}\n'.encode())

        elif msg.startswith("SET;POS"):
            parts = msg.split(';')
            self.x = float(parts[2])
            self.y = float(parts[3])
            self.theta = float(parts[4])
            self.ser.write(b'OK;POS;1\n')

    def stop(self):
        self.running = False
        self.thread.join()
        self.ser.close()


if __name__ == '__main__':
    try:
        sim = SimulatedPCB()
    except KeyboardInterrupt:
        sim.stop()
        print("Simulation stopped")
