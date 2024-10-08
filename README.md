
# CARLA NCAP Simulation Repository

This repository contains implementations of two tasks related to vehicle safety assessment using the CARLA simulator.

## Table of Contents
- [Task 1: CCRS NCAP Scenario](#task-1-ccrs-ncap-scenario)
- [Task 2: Lane Point Visualization](#task-2-lane-point-visualization)
- [Scripts Overview](#scripts-overview)
- [Requirements](#requirements)
- [How to Run](#how-to-run)
- [Output](#output)

## Task 1: CCRS NCAP Scenario
The goal of this task is to simulate the Car-to-Car Rear Stationary (CCRS) test in the CARLA simulator. The simulation involves the following:
- An ego vehicle (the tested car) starting 100 meters away from a stationary vehicle.
- The ego vehicle accelerates and brakes as it approaches the stationary vehicle.
- The scenario runs in synchronous mode with a fixed time-step of 0.05 seconds.

### Output Requirements
1. Visual output from a front-facing camera perspective of the ego vehicle.
2. A CSV file containing the following fields for each simulation step:
   - Ego Speed
   - Ego Acceleration
   - Ego Jerk
   - Relative distance to the stationary vehicle (from the ego car’s front bumper)
   - Stationary vehicle bounding box projected to the ego front-facing camera image (4 2-D pixel coordinates: top left, top right, bottom left, bottom right)

## Task 2: Lane Point Visualization
In this task, the original `manual_control.py` script from the CARLA repository has been modified to generate and visualize 20 points spaced 1 meter apart along both sides of the ego vehicle's lane during each simulation step. The points are visualized in the Pygame window.

## Scripts Overview
### Task 1 Scripts
- **main.py**: The main entry point for running the CCRS NCAP scenario.
- **controller.py**: Manages the control input for the ego vehicle.
- **bounding_box.py**: Generates bounding boxes for stationary vehicles.
- **sync_mode.py**: Implements a synchronous mode context manager for the simulation.
- **utility.py**: Contains utility functions for various tasks.

### Task 2 Script
- **manual_control.py**: Modified version of the original CARLA manual control script to visualize lane points.

## Requirements
To run the simulations, ensure you have the following:
- Python 3.7 or higher
- CARLA Simulator (version compatible with the scripts)
- Required Python packages (e.g., `numpy`, `pygame`, `pandas`, etc.)

You can install the required packages using:

```bash
pip install -r requirements.txt
```

## How to Run
1. **Clone the repository:**
   ```bash
   git clone <repository_url>
   cd <repository_directory>
   ```
   Or simply place folders Q1/Q2 in the carla/PythonAPI/ directory

2. **Run Task 1:**
   Execute the `main.py` script to start the CCRS NCAP simulation.
   ```bash
   python main.py
   ```

3. **Run Task 2:**
   Execute the modified `manual_control.py` script to visualize lane points.
   ```bash
   python manual_control.py
   ```

## Output
- The output for Task 1 will include a visual simulation window and a CSV file with the vehicle dynamics. A output example is presented in dir /_out.
- For Task 2, points will be visualized in the Pygame window during the simulation.


