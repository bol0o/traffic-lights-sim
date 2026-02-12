import subprocess
import struct
import time
import sys
import os

# Shares protocol.h structure
CMD_ADD_VEHICLE = 1
CMD_STEP = 2
CMD_STOP = 99

NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CORE_DIR = os.path.dirname(SCRIPT_DIR)
C_BINARY_PATH = os.path.join(CORE_DIR, 'core', 'bin', 'traffic_sim')

class TrafficSimulator:
    def __init__(self):
        if not os.path.exists(C_BINARY_PATH):
            raise FileNotFoundError(f"Could not find '{C_BINARY_PATH}'. Did you run 'make'")

        self.proc = subprocess.Popen(
            [C_BINARY_PATH],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=sys.stderr
        )
        print(f"C simulator running (PID: {self.proc.pid})")

    def add_vehicle(self, vehicle_id, start_road, end_road, arrival_time):
        print(f" -> [PY] Adding vehicle: {vehicle_id} ({start_road}->{end_road})")
        
        # 1. Header (1 byte: command type)
        # 'B' = unsigned char (1 byte)
        header = struct.pack('<B', CMD_ADD_VEHICLE)

        # 2. Payload (ayloadAddVehicle struct)
        # '32s' = string 32 bytes
        # 'B'   = unsigned char (start)
        # 'B'   = unsigned char (end)
        # 'I'   = unsigned int (4 bytes, time)
        vehicle_id_bytes = vehicle_id.encode('utf-8')
        payload = struct.pack('<32sBBI', vehicle_id_bytes, start_road, end_road, arrival_time)

        self.proc.stdin.write(header + payload)
        self.proc.stdin.flush()

    def step(self):
        self.proc.stdin.write(struct.pack('<B', CMD_STEP))
        self.proc.stdin.flush()

        # 2. Receive response (ResponseStep struct)
        # 'I' = uint32_t (current_step) -> 4 bytes
        # 'H' = uint16_t (vehicles_left) -> 2 bytes
        # Total: 6 bytes
        RESPONSE_SIZE = 6
        data = self.proc.stdout.read(RESPONSE_SIZE)

        if not data:
            raise RuntimeError("C process did not respond")

        current_step, cars_left_count = struct.unpack('<IH', data)
        
        print(f" <- [C]  Step: {current_step}, Cars that left the intersection: {cars_left_count}")
        return current_step

    def close(self):
        print("Exiting simulation...")
        try:
            self.proc.stdin.write(struct.pack('<B', CMD_STOP))
            self.proc.stdin.flush()
            self.proc.wait(timeout=1)
        except:
            self.proc.kill()
        print("Done")

if __name__ == "__main__":
    try:
        sim = TrafficSimulator()

        sim.add_vehicle("car_1", NORTH, SOUTH, 0)
        sim.add_vehicle("car_2", EAST,  WEST,  2)
        sim.add_vehicle("car_3", NORTH, EAST,  5) # Left turn

        for i in range(10):
            sim.step()
            time.sleep(0.1)

        sim.close()

    except Exception as e:
        print(f"Błąd: {e}")