# Traffic Lights Simulation

## Setup

**Prerequisites**

- GCC Compiler (for C core)
- Python 3.x
- Make
- Numpy (if running optimization algorithm)

**Building and running**

1. **Compile C code**:
```bash
cd core
make
```

You can also run `make test` to run tests.

2. **Run simulation:**

The simulation requires an input JSON file and an output path.
You can use `python3 optimize_timings.py` to generate test scenarios.

```bash
python3 pc-simulation/run_simulation.py input.json output.json
```
3. **(Optional) Run Optimizer / Benchmarks**
```bash
python3 pc-simulation/optimize_timings.py --optimize
```
## Project Structure

```text
├── assets/                     # Media for README
├── core/                       # Traffic Lights Simulation
│   ├── bin/                    # Compiled PC binaries
│   ├── lib/                    # Queue logic
│   ├── tests/                  # C unit tests
│   ├── main_pc.c               # Entry point for PC-based simulation
│   ├── makefile                # Build system for the PC executable
│   ├── protocol.h              # Shared protocol definiton
│   ├── traffic_fsm.c           # FSM implementation
│   └── traffic_fsm.h
├── firmware_stm32/             # STM32 project
│   └── ...
├── optimization_results/       # Results from algorithm optimizations
├── pc-simulation/              # Python Wrappers & Tools
│   ├── optimize_timings.py     # Parameter grid-search and cost optimization script
│   └── run_simulation.py       # Master controller
├── .gitignore                  
└── README.md
```

## Design assumptions

- **Right hand traffic**: The routing and lane logic are designed based on standard right-hand traffic rules.

- **Intersection geometry**: The model represents a standard 4-way intersection (North, East, South, West). Each road is assumed to have two dedicated lanes: one for **Straight/Right** movements and one for **Left** turns.

- **Two lane exits**: To maximize throughput, it is assumed that all exit roads have at least two lanes. This allows for simultaneous non-conflicting movements, such as a U-Turn entering the inner lane while a vehicle executing a Right Turn on Red (Green Arrow) enters the outer lane of the same road.

- **No mid-intersection blocking**: There is no "box junction" logic; vehicles never stop or wait in the middle of the intersection to complete a turn.

- **No collision**: The simulation assumes that vehicles occupy zero physical space within the center of the intersection. Once a vehicle is dequeued, it is assumed to exit the intersection instantly within that single simulation step.

- **Discrete time**: Each step command in the input JSON represents a fixed unit of time (1 step). All vehicle movements and state transitions are calculated relative to these discrete intervals.

## Algorithm overview

**Phase Sequencing**

The algorithm rotates through four primary green phases to cover all 4-way intersection movements:

1. **North-South Straight/Right**: Green light for through traffic and right turns from North and South.
2. **North-South Left**: Dedicated protected turn for vehicles heading East/West.
3. **East-West Straight/Right**: Green light for through traffic from East and West roads.
4. **East-West Left**: Dedicated protected turn for vehicles heading North/South.

**Optimizations**

The following optimizations were implemented:

- **Dynamic Phase Skipping**: Before entering any phase, the controller checks the real-time occupancy of the corresponding lanes. If no vehicles are detected (0 vehicles in queue), the phase is skipped entirely, moving immediately to the next road or direction to eliminate idle time (however, there is a limit on max number of skips in a row to prevent starvation)

- **Green Time Extension**: When the base green time expires, the algorithm analyzes the remaining queue length. And is able to extend green time if there are cars present.

- **Green right arrow**: For non-colliding directions.

**Algorithm params**

- Green straight/right time
- Green left time 
- Green extension threshold
- Maximum green extension time
- Phase skip limit

## Optimizing params

To determine the parameters at which the algorithm performs best, I used three metrics.

- **Average Waiting Time (AWT)** - mean time vehicles spend at intersection
- **Maximum Waiting Time (MAX)** - worst-case delay (fairness indicator)
- **Left-Turn Waiting Time (LEFT)** - average delay for left-turning vehicles

And 4 policies that prioritize these metrics differently:
- **Balanced**: Equal focus on all metrics
- **Fairness**: 2.0 weight on maximum wait to prevent starvation
- **Throughput**: Prioritizes average wait and throughput
- **Left_friendly**: 2.0 weight on left-turn waiting times

I tested 9 traffic patterns:

**6 normal traffic scenarios**
- steady: Uniform 10% flow from all directions
- rush: N-S corridor congestion (40% vs 5% others)
- ghost: Very sparse traffic (2%)
- asymmetric: Heavy load from single direction
- burst: Sudden 60% surge mid-simulation
- left_heavy: 70% left-turn bias

**3 jam scenarios**
- extreme_rush: Heavy N-S congestion testing the starvation limits of cross-traffic
- left_turn_jam: Overwhelming amount of left-turning vehicles
- all_directions_jam: Saturated intersection on all lanes

The optimizer performs a grid search across the parameter space:

```
ST (Straight Green Time)     : 4-18s (step 2)
LT (Left-Turn Green Time)    : 3-13s (step 2)  
EXT_T (Extension Threshold)  : 2-5 vehicles
MAX_EXT (Max Extensions)     : 3-15 steps
SKIP_L (Skip Limit)          : 1-3 cycles
```

All results were normalized, using the 80th percentile to ensures fair comparison across different traffic intensities.
```python
norm_avg = np.percentile(all_avg_wait_times, 80)
norm_max = np.percentile(all_max_wait_times, 80)
norm_left = np.percentile(all_left_wait_times, 80)
```

Cost function was calculated for each policy, scenario and parameters.

```python
cost = (awt_weight * (avg_wait / norm_avg) +
        max_weight * (max_wait / norm_max) +
        left_weight * (left_wait / norm_left))
```

### Optimization results

I tested 5 different version of my algorithm:
1. Fixed timing, all phases run
2. Phase skipping when empty
3. Green light extension
4. Green arrow for right turns
5. Adaptive limits (removed)

And these were the results:

**Normal traffic scenarios**

| Version | Version features | Improvement (AWT) vs V1 |
| ------- | ---------------- | ----------------------- |
| V1 | Static cycle with fixed timers | 0% (baseline) |
| V2 | Skips phases with zero vehicle occupancy | **~3%** (AWT 34 → 33) |
| V3 | Dynamically extends green light for detected queues | **~21%** (AWT 34 → 27) |
| V4 | Permissive right turns on red | **~32%** (AWT 34 → 23) |
| V5 (removed) | Dynamic road-specific limits (introduced instability) | **~21%** (Regression vs V4) |
| V6 | Optimization for all 5 parameters | **~32%** (cost J improved by 20% vs V4) |

**Jam traffic scenarios**

| Version | Version features | Improvement (MAX) vs V1 |
| ------- | ---------------- | ----------------------- |
| V1 | Static cycle with fixed timers | 0% (baseline) |
| V2 | Skips phases with zero vehicle occupancy | 0% (MAX 304 remained fixed) |
| V3 | Dynamically extends green light for detected queues | **~38%** (MAX 304 → 187) |
| V4 | Permissive right turns on red | **~39%** (MAX 304 → 185) |
| V5 (removed) | Dynamic road-specific limits (introduced instability) | **~17%** (Significant regression vs V4) |
| V6 | Optimization for all 5 parameters | **~35%** (Small regression, but best global normalized cost J) |

You can find screenshots for all tests in optimization_results folder.

### Optimal configuration

After testing ~380,000 simulations the search found optimum for the **Throughput** policy:

* **Base Straight Time:** 4 steps
* **Base Left Time:** 3 steps
* **Extension Threshold:** 1 vehicle
* **Max Extension:** 15 steps
* **Skip Limit:** 2 cycles

Instead of long, fixed green phases, the algorithm proved that a short cycle and extension for each car strategy works best. The intersection cycles very fast when empty, minimizing idle time. However, it is very sensitive (EXT_T = 1) and will hold the green light for up to 19 seconds (4s + 15s) if a continuous stream of cars is detected.

**Other insights**
1. **Over adaptation (V5 vs V6):** Attempting to make the system better by dynamically altering the maximum allowed green time based on real-time traffic pressure (V5) actually caused a regression in heavy traffic. It created a Starvation Effect by constantly favoring the busiest road, cross-traffic was effectively blocked, causing the Maximum Wait Time to spike by 17%. V6 proved that having strict, hard-coded limits (MAX_EXT) combined with short base cycles provides better fairness and throughput.

2. **Permissive turns (V4):** Introducing green right-turn arrows during non-conflicting left-turn phases (V4) yielded a massive 32% improvement in Average Wait Time. This highlights that optimizing spatial lane utilization is just as important as optimizing time.

3. **Ghost Traffic Mitigation (V2 & V3):** 
While Phase Skipping (V2) only provided a 3% improvement on its own, it became powerful when combined with Green Extensions (V3). Skipping empty roads saves time, which is then dynamically reallocated to busy roads via extensions.

## STM32 Demo

![demo gif](./assets/demo.gif)

This demo shows an STM32 Nucleo-G0B1RE microcontroller running the algorithm and displaying the real-time intersection state on a breadboard.

**How it works**

1. Python reads the input.json file and acts as the master controller.
2. Encodes the simulation steps into binary structs and streams them via UART to the STM32.
3. The STM32 receives the payload, processes the FSM step, and updates the physical GPIOs using the STM32 HAL library.
4. The microcontroller sends a binary response back to the Python, containing the intersection state and IDs of vehicles that successfully left the queue.

*Note: The Python wrapper for the hardware simulation is almost identical to the PC-based simulation thanks to the shared protocol.h. The only difference is swapping standard I/O pipes for a Serial COM port access.*

**Hardware Mapping**

To represent the 2-lane intersection logic on a limited hardware setup, the logic was mapped as follows:
RGB LEDs Represent the active light for a given direction. Because the FSM tracks Straight and Left turns independently - green illuminates for main green and permissive green.

Blue/Orange LEDs (Queue length): Act as a real-time visualization of the queues.
- 1 LED ON = Queue has at least 1 vehicle.
- 2 LEDs ON = Queue has 2 or more vehicles waiting.

