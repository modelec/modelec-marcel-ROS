import argparse

import serial
import time
import math
import threading

class SimulatedPCB:
    def __init__(self, port='/dev/pts/6', baud=115200):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.running = True
        self.last_update = time.time()

        # État simulé
        self.servos = {}  # {id: pos_index}
        self.servo_angles = {}  # {id: {pos_index: angle}}
        self.relais = {1: 0, 2: 0, 3: 0}
        self.ascenseur_pos = "low"  # 'low', 'climb', 'high', 'desc'
        self.fin_course = {}  # à implémenter si besoin
        self.tirette_arm = False

        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def send_response(self, msg):
        print(f"[TX] {msg}")
        self.ser.write((msg + "\n").encode())

    def process_command(self, line):
        parts = line.strip().split(';')
        if not parts:
            return

        cmd = parts[0]

        if cmd == "GET":
            self.handle_get(parts)
        elif cmd == "SET":
            self.handle_set(parts)
        elif cmd == "MOV":
            self.handle_mov(parts)

    def handle_get(self, parts):
        if len(parts) != 3:
            return

        category, data = parts[1], parts[2]

        if category.startswith("SERVO") and data == "POS":
            sid = int(category[5:])
            pos = self.servos.get(sid, 0)
            self.send_response(f"SET;{category};{data};{pos}")
        elif category == "ASC" and data == "POS":
            self.send_response(f"SET;ASC;POS;{self.ascenseur_pos}")
        elif category.startswith("RELAY") and data == "STATE":
            rid = int(category[5:])
            state = self.relais.get(rid, 0)
            self.send_response(f"SET;{category};{data};{state}")

    def handle_set(self, parts):
        if len(parts) != 4:
            return

        category, data, val = parts[1], parts[2], parts[3]

        if category.startswith("SERVO") and data.startswith("POS"):
            sid = int(category[5:])
            pos_index = int(data[3:])
            angle = float(val)

            if sid not in self.servo_angles:
                self.servo_angles[sid] = {}
            self.servo_angles[sid][pos_index] = angle

            self.send_response(f"OK;{category};{data};{val}")
        elif category == "ASC" and data in ("HIGH", "LOW"):
            self.send_response(f"OK;ASC;{data};{val}")
        else:
            self.send_response(f"KO;{category};{data};{val}")

    def handle_mov(self, parts):
        if len(parts) != 3:
            return

        category, data = parts[1], parts[2]

        if category == "ASC":
            if data in ("HIGH", "LOW"):
                self.ascenseur_pos = data.lower()
                self.send_response(f"OK;{category};{data}")
            else:
                self.send_response(f"KO;{category};{data}")
        elif category.startswith("SERVO") and data.startswith("POS"):
            sid = int(category[5:])
            pos_index = int(data[3:])
            if sid in self.servo_angles and pos_index in self.servo_angles[sid]:
                self.servos[sid] = pos_index
                self.send_response(f"OK;{category};{data}")
            else:
                self.send_response(f"KO;{category};{data}")
        elif category == "RELAY":
            if data == "1":
                for r in self.relais:
                    self.relais[r] = 1
                self.send_response(f"OK;RELAY;1")
            elif data == "0":
                for r in self.relais:
                    self.relais[r] = 0
                self.send_response(f"OK;RELAY;0")
            else:
                self.send_response(f"KO;RELAY;{data}")
        else:
            self.send_response(f"KO;{category};{data}")

    def run(self):
        while self.running:
            if self.ser.in_waiting:
                msg = self.ser.readline().decode().strip()
                print(f"[RX] {msg}")
                self.process_command(msg)

            time.sleep(0.1)

    def stop(self):
        self.running = False
        self.thread.join()
        self.ser.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Simulated PCB")
    parser.add_argument('--port', type=str, default='/dev/pts/6', help='Serial port to use')
    args = parser.parse_args()

    sim = None
    try:
        sim = SimulatedPCB(port=args.port)
    except KeyboardInterrupt:
        if sim:
            sim.stop()
        print("Simulation stopped")
