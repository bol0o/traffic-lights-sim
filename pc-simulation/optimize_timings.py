import json
import random
import itertools
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple
import subprocess
import os
import struct

# ============ CONFIG ============
ROADS = ["north", "east", "south", "west"]
SEED = 42
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CORE_DIR = os.path.dirname(SCRIPT_DIR)
C_BINARY_PATH = os.path.join(CORE_DIR, 'core', 'bin', 'traffic_sim')

# Grid Search Ranges
# Note: To keep demonstration runtime reasonable, EXT_T, MAX_EXT and SKIP_L 
# are narrowed down to their historically best performing bounds.
ST_RANGE = range(4, 19, 2)
LT_RANGE = range(3, 13, 2)
EXT_THRESHOLD_RANGE = [1]
MAX_EXT_RANGE = [15]
SKIP_LIMIT_RANGE = [2]


# ============ DATA STRUCTURES ============
@dataclass
class TimingParams:
    """Lights timing params"""
    green_st: int
    green_lt: int
    yellow: int = 2
    all_red: int = 3
    ext_threshold: int = 3
    max_ext: int = 10
    skip_limit: int = 2

    def to_dict(self):
        return {
            'green_st': self.green_st,
            'green_lt': self.green_lt,
            'yellow': self.yellow,
            'all_red': self.all_red,
            'ext_threshold': self.ext_threshold, 
            'max_ext': self.max_ext,
            'skip_limit': self.skip_limit
        }


@dataclass
class ScenarioMetrics:
    """Metrics for single simulation run"""
    avg_wait: float
    max_wait: float
    throughput: int
    left_wait: float = 0.0

    def cost(self, awt=1.0, max=0.5, left=0.3,
             norm_avg=None, norm_max=None, norm_left=None) -> float:
        """
        Cost function with optional normalization
        If norms provided -> normalized values in [0,1]
        If no norms -> raw values
        """
        if norm_avg is not None and norm_max is not None and norm_left is not None:
            avg_norm = min(1.0, self.avg_wait / norm_avg) if norm_avg > 0 else 0
            max_norm = min(1.0, self.max_wait / norm_max) if norm_max > 0 else 0
            left_norm = min(1.0, self.left_wait / norm_left) if norm_left > 0 else 0
            return awt * avg_norm + max * max_norm + left * left_norm
        else:
            return awt * self.avg_wait + max * self.max_wait + left * self.left_wait


@dataclass
class Scenario:
    """Test scenario definition"""
    name: str
    steps: int
    prob_func: Callable
    left_bias: float = 0.25


# ============ SCENARIO DEFINITIONS ============

SCENARIOS = [
    Scenario("steady", 200, lambda s, r: 0.1, 0.25),
    Scenario("rush", 200, lambda s, r: 0.4 if r in ["north", "south"] else 0.05, 0.25),
    Scenario("ghost", 200, lambda s, r: 0.02, 0.25),
    Scenario("asymmetric", 200, lambda s, r: 0.4 if r == "north" else 0.05, 0.25),
    Scenario("burst", 200, lambda s, r: 0.6 if 50 <= s <= 100 else 0.05, 0.25),
    Scenario("left_heavy", 200, lambda s, r: 0.15, 0.7),
]

JAM_SCENARIOS = [
    Scenario("extreme_rush", 500, lambda s, r: 0.6 if r in ["north", "south"] else 0.3, 0.3),
    Scenario("left_turn_jam", 400, lambda s, r: 0.5, 0.8),
    Scenario("all_directions_jam", 300, lambda s, r: 0.7, 0.5),
]

# ============ UTILITY FUNCTIONS ============

def get_left_turn_target(start_road):
    idx = ROADS.index(start_road)
    return ROADS[(idx + 1) % 4]


def create_command_list(scenario: Scenario, seed=None):
    if seed is not None:
        random.seed(seed)
    else:
        random.seed(SEED)

    commands = []
    vehicle_count = 0

    for step in range(scenario.steps):
        for start in ROADS:
            prob = scenario.prob_func(step, start) if callable(scenario.prob_func) else scenario.prob_func
            if random.random() < prob:
                vehicle_count += 1

                if random.random() < scenario.left_bias:
                    end = get_left_turn_target(start)
                else:
                    possible_targets = [r for r in ROADS
                                      if r != start and r != get_left_turn_target(start)]
                    end = random.choice(possible_targets)

                commands.append({
                    "type": "addVehicle",
                    "vehicleId": f"v_{start[0]}_{vehicle_count}",
                    "startRoad": start,
                    "endRoad": end
                })

        commands.append({"type": "step"})

    return {"commands": commands, "name": scenario.name}


# ============ SIMULATION ENGINE ============

def run_single_simulation(scenario_data: dict, params: TimingParams) -> ScenarioMetrics:
    if not os.path.exists(C_BINARY_PATH):
        raise FileNotFoundError(f"Binary not found: {C_BINARY_PATH}")

    proc = subprocess.Popen(
        [C_BINARY_PATH],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    # Send config
    header = struct.pack('<B', 0)  # CMD_CONFIG
    payload = struct.pack('<IIIIIII',
                        params.green_st,
                        params.green_lt,
                        params.yellow,
                        params.all_red,
                        params.ext_threshold,
                        params.max_ext,
                        params.skip_limit)
    proc.stdin.write(header + payload)
    proc.stdin.flush()

    # Run simulation
    arrival_times = {}
    wait_times = []
    left_wait_times = []
    current_step = 0

    for cmd in scenario_data["commands"]:
        if cmd["type"] == "addVehicle":
            v_id = cmd["vehicleId"]
            arrival_times[v_id] = current_step
            start_id = ROADS.index(cmd["startRoad"])
            end_id = ROADS.index(cmd["endRoad"])

            header = struct.pack('<B', 1)  # CMD_ADD_VEHICLE
            v_id_bytes = v_id.encode('utf-8')
            payload = struct.pack('<32sBBI', v_id_bytes, start_id, end_id, current_step)
            proc.stdin.write(header + payload)
            proc.stdin.flush()

        elif cmd["type"] == "step":
            current_step += 1
            proc.stdin.write(struct.pack('<B', 2))  # CMD_STEP
            proc.stdin.flush()

            header_data = proc.stdout.read(11)
            if len(header_data) < 11:
                break

            _, _, _, _, _, _, v_count = \
                struct.unpack('<IBBBBBH', header_data)

            if v_count > 0:
                raw_ids = proc.stdout.read(v_count * 32)
                for i in range(v_count):
                    v_id = raw_ids[i*32:(i+1)*32].decode('utf-8').strip('\x00')
                    if v_id in arrival_times:
                        wait = current_step - arrival_times[v_id]
                        wait_times.append(wait)

                        # Check if it's left turn
                        cmd_data = next((c for c in scenario_data["commands"]
                                       if c.get("vehicleId") == v_id), None)
                        if cmd_data:
                            start = cmd_data["startRoad"]
                            end = cmd_data["endRoad"]
                            if get_left_turn_target(start) == end:
                                left_wait_times.append(wait)

    # Cleanup
    proc.stdin.write(struct.pack('<B', 99))  # CMD_STOP
    proc.stdin.flush()
    proc.wait(timeout=1)

    return ScenarioMetrics(
        avg_wait=sum(wait_times) / len(wait_times) if wait_times else 0,
        max_wait=max(wait_times) if wait_times else 0,
        throughput=len(wait_times),
        left_wait=sum(left_wait_times) / len(left_wait_times) if left_wait_times else 0
    )

# ============ GLOBAL NORMALIZATION ============

def calculate_global_norms(scenarios: List[Scenario]) -> Tuple[float, float, float]:
    print("\n[1/3] Calculating global normalization factors (80th percentile)...")
    
    all_avg = []
    all_max = []
    all_left = []

    for scenario in scenarios:
        print(f" -> {scenario.name}")
        scenario_data = create_command_list(scenario, seed=SEED)
        
        current_search = itertools.product(
            ST_RANGE, LT_RANGE, EXT_THRESHOLD_RANGE, MAX_EXT_RANGE, SKIP_LIMIT_RANGE
        )

        for st, lt, eth, mext, skip in current_search:
            params = TimingParams(
                green_st=st, green_lt=lt, 
                ext_threshold=eth, max_ext=mext, skip_limit=skip
            )
            metrics = run_single_simulation(scenario_data, params)

            all_avg.append(metrics.avg_wait)
            all_max.append(metrics.max_wait)
            all_left.append(metrics.left_wait)
            
    import numpy as np
    
    norm_avg = np.percentile(all_avg, 80) if all_avg else 50.0
    norm_max = np.percentile(all_max, 80) if all_max else 200.0
    norm_left = np.percentile(all_left, 80) if all_left else 60.0

    print(f"\nNormalization factors (80 percentile):")
    print(f" AWT:  {norm_avg:6.1f} steps")
    print(f" MAX:  {norm_max:6.1f} steps")
    print(f" LEFT: {norm_left:6.1f} steps")
    print(f" Samples: AWT={len(all_avg)}, MAX={len(all_max)}, LEFT={len(all_left)}")

    return norm_avg, norm_max, norm_left


# ============ GRID SEARCH WITH GLOBAL NORMS ============

def grid_search(
    scenario: Scenario,
    global_norms: tuple,
    weights: dict
) -> tuple:
    norm_avg, norm_max, norm_left = global_norms
    scenario_data = create_command_list(scenario, seed=SEED)

    print(f"\nGrid search: {scenario.name}")
    print(f"Using global norms: AWT={norm_avg:.1f}, MAX={norm_max:.1f}, LEFT={norm_left:.1f}")

    best_cost = float('inf')
    best_params = None
    results = []
    
    search_space = itertools.product(
        ST_RANGE, 
        LT_RANGE, 
        EXT_THRESHOLD_RANGE, 
        MAX_EXT_RANGE, 
        SKIP_LIMIT_RANGE
    )

    for st, lt, eth, mext, skip in search_space:
        params = TimingParams(
            green_st=st, 
            green_lt=lt, 
            ext_threshold=eth, 
            max_ext=mext, 
            skip_limit=skip
        )
        
        metrics = run_single_simulation(scenario_data, params)

        cost = metrics.cost(
            awt=weights['awt'],
            max=weights['max'],
            left=weights['left'],
            norm_avg=norm_avg,
            norm_max=norm_max,
            norm_left=norm_left
        )

        results.append({
            'st': st,
            'lt': lt,
            'eth': eth,
            'mext': mext,
            'skip': skip,
            'cost': cost,
            'avg_wait': metrics.avg_wait,
            'max_wait': metrics.max_wait,
            'left_wait': metrics.left_wait,
            'throughput': metrics.throughput
        })

        if cost < best_cost:
            best_cost = cost
            best_params = params

        if len(results) % 5 == 0:
            print(f" ST={st:2d}, LT={lt:2d} → J={cost:.3f}")

    print(f"\n{scenario.name} optimum: ST={best_params.green_st}s, LT={best_params.green_lt}s, J={best_cost:.3f}")
    return best_params, best_cost, results

def multi_scenario_optimization(
    scenarios: Optional[List[Scenario]] = None,
    weights: Optional[dict] = None
) -> dict:
    if scenarios is None:
        scenarios = SCENARIOS
    if weights is None:
        weights = {'awt': 1.0, 'max': 0.5, 'left': 0.3}

    print("Full grid search")
    print(f"Weights: AWT={weights['awt']}, MAX={weights['max']}, LEFT={weights['left']}")

    global_norms = calculate_global_norms(scenarios)
    norm_avg, norm_max, norm_left = global_norms

    print("1. Individual scenario optima")
    
    scenario_optima = {}
    for scenario in scenarios:
        best_params, best_cost, _ = grid_search(
            scenario, global_norms, weights
        )
        scenario_optima[scenario.name] = {
            'params': best_params.to_dict(),
            'cost': best_cost
        }

    print("2: Compromise search")
    
    compromise_results = []

    search_space = itertools.product(
        ST_RANGE, LT_RANGE, EXT_THRESHOLD_RANGE, MAX_EXT_RANGE, SKIP_LIMIT_RANGE
    )

    for st, lt, eth, mext, skip in search_space:
        params = TimingParams(
            green_st=st, 
            green_lt=lt, 
            ext_threshold=eth, 
            max_ext=mext, 
            skip_limit=skip
        )
        
        total_cost = 0.0
        scenario_costs = {}

        for scenario in scenarios:
            scenario_data = create_command_list(scenario, seed=SEED)
            metrics = run_single_simulation(scenario_data, params)

            cost = metrics.cost(
                awt=weights['awt'],
                max=weights['max'],
                left=weights['left'],
                norm_avg=norm_avg,
                norm_max=norm_max,
                norm_left=norm_left
            )
            
            total_cost += cost
            scenario_costs[scenario.name] = cost

        avg_cost = total_cost / len(scenarios)
        compromise_results.append({
            'st': st,
            'lt': lt,
            'eth': eth,
            'mext': mext,
            'skip': skip,
            'avg_cost': avg_cost,
            'total_cost': total_cost,
            'scenario_costs': scenario_costs
        })

        if len(compromise_results) % 20 == 0:
            print(f"Tested {len(compromise_results)} combinations... Current Best J={min(c['avg_cost'] for c in compromise_results):.3f}")

    best = min(compromise_results, key=lambda x: x['avg_cost'])

    print("Optimal compromise config")
    print(f"   GREEN STRAIGHT: {best['st']}s")
    print(f"   GREEN LEFT:     {best['lt']}s")
    print(f"   EXT_THRESHOLD:  {best['eth']}")
    print(f"   MAX_EXTENSION:  {best['mext']} steps")
    print(f"   SKIP_LIMIT:     {best['skip']} cycles")
    print(f"\n   Average normalized cost J = {best['avg_cost']:.3f}")
    print("\n   Per-scenario costs:")
    for sc_name, sc_cost in best['scenario_costs'].items():
        print(f"     {sc_name:12s}: J={sc_cost:.3f}")

    return {
        'global_norms': {
            'avg': norm_avg, 'max': norm_max, 'left': norm_left
        },
        'scenario_optima': scenario_optima,
        'compromise': best,
        'all_compromises': compromise_results
    }


# ============ BENCHMARK GENERATION ============

def save_benchmarks():
    print("\nGenerating benchmarks")

    for scenario in SCENARIOS + JAM_SCENARIOS:
        data = create_command_list(scenario, seed=SEED)
        filename = f"bench_{scenario.name}.json"
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"{filename:20s} ({len(data['commands']):4d} commands)")


# ============ MAIN ============

if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "--generate-only":
        save_benchmarks()

    elif len(sys.argv) > 1 and sys.argv[1] == "--optimize":
        policies = {
            'balanced': {'awt': 1.0, 'max': 0.5, 'left': 0.3},
            'fairness': {'awt': 0.7, 'max': 2.0, 'left': 0.5},
            'throughput': {'awt': 1.0, 'max': 0.3, 'left': 0.2},
            'left_friendly': {'awt': 0.8, 'max': 0.4, 'left': 2.0},
        }

        all_results = {}

        for policy_name, weights in policies.items():
            print(f"Using policy: {policy_name.upper()}")

            results = multi_scenario_optimization(
                scenarios=SCENARIOS,
                weights=weights
            )
            all_results[policy_name] = results

        print("\n" + "=" * 60)
        print("📊 OPTIMIZATION SUMMARY")
        print("=" * 60)

        for policy_name, results in all_results.items():
            comp = results['compromise']
            norms = results['global_norms']
            print(f"\n{policy_name.upper()}:")
            
            print(f"  ST = {comp['st']}s, LT = {comp['lt']}s")
            print(f"  EXT_T = {comp['eth']}, MAX_EXT = {comp['mext']}, SKIP_L = {comp['skip']}")
            
            print(f"  Avg normalized cost = {comp['avg_cost']:.3f}")
            print(f"  (Norms: AWT={norms['avg']:.0f}, MAX={norms['max']:.0f}, LEFT={norms['left']:.0f})")

    else:
        save_benchmarks()
        print("\nRun with --optimize to perform optimization")
        print("Example: python3 generate_benchmarks.py --optimize")