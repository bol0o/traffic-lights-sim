import subprocess
import struct
import json
import sys
import os
from typing import Any, Dict, Optional

# Shares protocol.h structure
CMD_CONFIG = 0
CMD_ADD_VEHICLE = 1
CMD_STEP = 2
CMD_STOP = 99

NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3
ROAD_MAP = {"north": 0, "east": 1, "south": 2, "west": 3}

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CORE_DIR = os.path.dirname(SCRIPT_DIR)
C_BINARY_PATH = os.path.join(CORE_DIR, 'core', 'bin', 'traffic_sim')

class TrafficSimulator:
    """
    Manages the lifecycle and binary communication with the C FSM core.
    """

    def __init__(self, config: Optional[Dict[str, int]] = None):
        if not os.path.exists(C_BINARY_PATH):
            raise FileNotFoundError(f"Could not find '{C_BINARY_PATH}'. Did you run 'make'?")

        self.proc = subprocess.Popen(
            [C_BINARY_PATH],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=sys.stderr
        )
        
        if config:
            self.send_config(config)
        else:
            default_config = {
                'green_st': 4,
                'green_lt': 3,
                'yellow': 1,
                'all_red': 3,
                'ext_threshold': 1,
                'max_ext': 15,
                'skip_limit': 2
            }
            self.send_config(default_config)
            
        print(f"C simulator running (PID: {self.proc.pid})")

    def send_config(self, config: Dict[str, int]) -> None:
        print(f" -> [PY] Sending config: ST={config['green_st']}s, LT={config['green_lt']}s, Y={config['yellow']}s, AR={config['all_red']}s")
        
        header = struct.pack('<B', CMD_CONFIG)
        payload = struct.pack('<IIIIIII', 
                            config['green_st'],
                            config['green_lt'], 
                            config['yellow'],
                            config['all_red'],
                            config['ext_threshold'],
                            config['max_ext'],
                            config['skip_limit'])
        
        self.proc.stdin.write(header + payload)
        self.proc.stdin.flush()

    def add_vehicle(self, vehicle_id: str, start_road: str, end_road: str, arrival_time: int) -> None:
        """Encodes vehicle data and pushes it to the MCU queues."""
        start_id = ROAD_MAP[start_road]
        end_id = ROAD_MAP[end_road]
        
        # 1. Header (1 byte: command type)
        # 'B' = unsigned char (1 byte)
        header = struct.pack('<B', CMD_ADD_VEHICLE)

        # 2. Payload (ayloadAddVehicle struct)
        # '32s' = string 32 bytes
        # 'B' = unsigned char (start)
        # 'B' = unsigned char (end)
        # 'I' = unsigned int (4 bytes, time)
        vehicle_id_bytes = vehicle_id.encode('utf-8')
        payload = struct.pack('<32sBBI', vehicle_id_bytes, start_id, end_id, arrival_time)

        self.proc.stdin.write(header + payload)
        self.proc.stdin.flush()

    def step(self) -> Dict[str, Any]:
        self.proc.stdin.write(struct.pack('<B', CMD_STEP))
        self.proc.stdin.flush()

        HEADER_SIZE = 11
        header_data = self.proc.stdout.read(HEADER_SIZE)

        if not header_data:
            raise RuntimeError("C process did not respond")

        step_idx, _, _, _, _, _, v_count = struct.unpack('<IBBBBBH', header_data)
        
        left_vehicles = []
        if v_count > 0:
            raw_ids = self.proc.stdout.read(v_count * 32)
            for i in range(v_count):
                v_id = raw_ids[i*32 : (i+1)*32].decode('utf-8').strip('\x00')
                left_vehicles.append(v_id)

        return {"step": step_idx, "leftVehicles": left_vehicles}

    def close(self) -> None:
        """Gracefully terminates the C process, with a forced kill fallback."""
        try:
            self.proc.stdin.write(struct.pack('<B', CMD_STOP))
            self.proc.stdin.flush()
            self.proc.wait(timeout=1)
        except:
            self.proc.kill()

def run_simulation(input_file: str, output_file: str, timing_params: Optional[Dict[str, int]] = None) -> Dict[str, float]:
    """
    Main execution loop. Parses the scenario, steps the FSM, 
    calculates performance metrics, and dumps the output JSON.
    """
    with open(input_file, 'r') as f:
        scenario = json.load(f)

    sim = TrafficSimulator(timing_params)
    output_data = {"stepStatuses": []}
    
    arrival_times = {}
    wait_times = []
    current_step = 0

    print(f"Starting simulation from {input_file}...")

    for cmd in scenario.get("commands", []):
        if cmd["type"] == "addVehicle":
            v_id = cmd["vehicleId"]
            arrival_times[v_id] = current_step
            sim.add_vehicle(v_id, cmd["startRoad"], cmd["endRoad"], current_step)
        
        elif cmd["type"] == "step":
            current_step += 1
            result = sim.step()
            
            for v_id in result["leftVehicles"]:
                if v_id in arrival_times:
                    wait = current_step - arrival_times[v_id]
                    wait_times.append(wait)
            
            output_data["stepStatuses"].append({"leftVehicles": result["leftVehicles"]})

    sim.close()

    with open(output_file, 'w') as f:
        json.dump(output_data, f, indent=4)

    if wait_times:
        avg_wait = sum(wait_times) / len(wait_times)
        max_wait = max(wait_times)
        print("\n --- PERFORMANCE METRICS ---")
        print(f"   Avg Wait Time: {avg_wait:.2f} steps")
        print(f"   Max Wait Time: {max_wait} steps")
        print(f"   Throughput:    {len(wait_times)} vehicles")
    else:
        print("\n --- PERFORMANCE METRICS ---")
        print("   No vehicles processed.")
    
    print(f"\n[PY] Simulation finished. Output saved to {output_file}")
    
    metrics = {
        'avg_wait': sum(wait_times) / len(wait_times) if wait_times else 0,
        'max_wait': max(wait_times) if wait_times else 0,
        'throughput': len(wait_times)
    }
    
    return metrics

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 run_simulation.py <input.json> <output.json>")
        sys.exit(1)
    
    run_simulation(sys.argv[1], sys.argv[2])