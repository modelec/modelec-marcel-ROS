import argparse
import serial
import threading
import time

class SimulatedPCB:
    def __init__(self, port='/tmp/ACTION_SIM', baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.running = True

        # State identical to STM32 getters
        self.servo_angles = {i: 0.0 for i in range(16)}     # angle as float
        self.relay_state = {i: 0 for i in range(16)}        # 0/1
        self.tirette_armed = False

        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def tx(self, msg):
        print(f"[TX] {msg.strip()}")
        self.ser.write((msg + "\n").encode())

    # ----------------------------------------------------------------------
    # STM32-LIKE FUNCTIONS
    # ----------------------------------------------------------------------
    def getServoAngle(self, sid):
        return float(self.servo_angles.get(sid, 0.0))

    def getRelayState(self, rid):
        return int(self.relay_state.get(rid, 0))

    def moveServo(self, sid, angle):
        self.servo_angles[sid] = float(angle)

    def moveRelay(self, rid, state):
        self.relay_state[rid] = 1 if int(state) else 0

    # ----------------------------------------------------------------------
    # COMMAND PARSER (as close as possible to STM32 version)
    # ----------------------------------------------------------------------
    def process(self, line):
        print(f"[RX] {line}")
        parts = line.split(";")
        if len(parts) < 2:
            return

        cmd = parts[0]

        if cmd == "GET":
            self.handle_get(parts)
        elif cmd == "SET":
            self.handle_set(parts)
        elif cmd == "MOV":
            self.handle_mov(parts)
        elif cmd == "ACK":
            # simulate ACK reception ignore for now
            pass

    # ----------------------------------------------------------------------
    # GET handler (SERVO POS / RELAY STATE)
    # ----------------------------------------------------------------------
    def handle_get(self, p):
        # GET;SERVO;POS;n;id...
        if p[1] == "SERVO" and p[2] == "POS":
            if len(p) < 4:
                self.tx("KO;SET;SERVO;POS")
                return

            n = int(p[3])
            if n <= 0:
                self.tx("SET;SERVO;POS;0")
                return

            resp = f"SET;SERVO;POS;{n}"
            idx = 4
            for _ in range(n):
                if idx >= len(p):
                    self.tx("KO;SET;SERVO;POS")
                    return
                sid = int(p[idx])
                idx += 1
                angle = self.getServoAngle(sid)
                resp += f";{sid};{angle:.4f}"

            self.tx(resp)
            return

        # GET;RELAY;STATE;n;id...
        if p[1] == "RELAY" and p[2] == "STATE":
            if len(p) < 4:
                self.tx("KO;SET;RELAY;STATE")
                return
            n = int(p[3])
            if n <= 0:
                self.tx("SET;RELAY;STATE;0")
                return

            resp = f"SET;RELAY;STATE;{n}"
            idx = 4
            for _ in range(n):
                if idx >= len(p):
                    self.tx("KO;SET;RELAY;STATE")
                    return
                rid = int(p[idx])
                idx += 1
                state = self.getRelayState(rid)
                resp += f";{rid};{state}"

            self.tx(resp)
            return

    # ----------------------------------------------------------------------
    # SET handler (only TIR;ARM)
    # ----------------------------------------------------------------------
    def handle_set(self, p):
        if p[1] == "TIR" and p[2] == "ARM":
            if len(p) < 4:
                self.tx("KO;TIR;ARM")
                return

            val = int(p[3])
            self.tirette_armed = bool(val)
            self.tx(f"OK;TIR;ARM;{val}")
            return

        self.tx(f"KO;{p[1]};{p[2]}")

    # ----------------------------------------------------------------------
    # MOV handler (SERVO multi / RELAY multi)
    # ----------------------------------------------------------------------
    def handle_mov(self, p):
        # MOV;SERVO;n;id;angle;id;angle...
        if p[1] == "SERVO":
            n = int(p[2])
            idx = 3

            resp = f"OK;SERVO;{n}"
            for _ in range(n):
                if idx + 1 >= len(p):
                    self.tx(f"KO;MOV;SERVO;{p[2]}")
                    return

                sid = int(p[idx])
                ang = float(p[idx+1])
                idx += 2

                self.moveServo(sid, ang)
                resp += f";{sid};{ang}"

            self.tx(resp)
            return

        # MOV;RELAY;n;id;state...
        if p[1] == "RELAY":
            n = int(p[2])
            idx = 3

            resp = f"OK;RELAY;{n}"
            for _ in range(n):
                if idx + 1 >= len(p):
                    self.tx(f"KO;MOV;RELAY;{p[2]}")
                    return

                rid = int(p[idx])
                st = int(p[idx+1])
                idx += 2

                self.moveRelay(rid, st)
                resp += f";{rid};{st}"

            self.tx(resp)
            return

        self.tx(f"KO;MOV;{p[1]}")

    # ----------------------------------------------------------------------
    # Loop
    # ----------------------------------------------------------------------
    def run(self):
        buf = b""
        while self.running:
            try:
                data = self.ser.read(64)
                if data:
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self.process(line.decode().strip())
            except Exception as e:
                print(f"Serial error: {e}")
                time.sleep(0.2)

    def stop(self):
        self.running = False
        self.thread.join()
        self.ser.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=str, default="/tmp/ACTION_SIM")
    args = parser.parse_args()
    sim = None
    try:
        sim = SimulatedPCB(args.port)
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        if sim:
            sim.stop()
        print("Stopped.")

# socat -d -d pty,raw,echo=0,link=/tmp/SIM_ACTION pty,raw,echo=0,link=/tmp/USB_ACTION
# python3 simulated_pcb/action.py --port /tmp/SIM_ACTION
# socat -d -d pty,raw,echo=0,link=/tmp/SIM_ODO pty,raw,echo=0,link=/tmp/USB_ODO
# python3 simulated_pcb/action.py --port /tmp/SIM_ODO
