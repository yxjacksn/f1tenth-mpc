# F1TENTH MPC Controller

MPC-based autonomous controller for the levine_blocked map in the F1TENTH simulator.

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- F1TENTH simulator (`f1tenth_gym_ros`) already built in a workspace (e.g. `sim_ws`)

## Setup (one time)
```bash
# 1. Install dependencies
pip3 install casadi --break-system-packages

# 2. Clone this repo into a clean workspace
mkdir -p ~/ese6150/mpc_ws/src
cd ~/ese6150/mpc_ws/src
git clone git@github.com:yxjacksn/f1tenth-mpc.git mpc_pkg

# 3. Build
cd ~/ese6150/mpc_ws
source /opt/ros/humble/setup.bash
source ~/ese6150/sim_ws/install/setup.bash
colcon build --symlink-install

# 4. Update the waypoint path
# Edit mpc_ws/src/mpc_pkg/config/mpc_params.yaml
# Change the waypoint_csv line to your absolute path:
#   waypoint_csv: "/home/YOUR_USERNAME/ese6150/mpc_ws/src/mpc_pkg/data/waypoints.csv"
```

## Running

You need two terminals.

**Terminal 1 — Simulator:**
```bash
source /opt/ros/humble/setup.bash
source ~/ese6150/sim_ws/install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

**Terminal 2 — MPC Controller:**
```bash
source /opt/ros/humble/setup.bash
source ~/ese6150/sim_ws/install/setup.bash
source ~/ese6150/mpc_ws/install/setup.bash
ros2 launch mpc_pkg mpc.launch.py
```

The car should start driving autonomously within a few seconds.

## Visualization (optional)

If you have Foxglove Desktop on your host machine:

1. Connect to `ws://<VM_IP>:8765`
2. Add a 3D panel
3. Set frame to `map`
4. Enable topics: `/map`, `/scan`, `/tf`, `/mpc_viz`, `/mpc_waypoints`

- Green line = MPC predicted trajectory
- Blue line = reference path being tracked
- Red dots = full waypoint track

## Key files

| File | What it does |
|------|-------------|
| `mpc_pkg/mpc_core.py` | CasADi/IPOPT nonlinear MPC solver |
| `mpc_pkg/node.py` | ROS 2 node (odom subscriber, drive publisher) |
| `mpc_pkg/waypoint_utils.py` | Waypoint loading + nearest-point tracking |
| `mpc_pkg/viz.py` | MarkerArray visualization helpers |
| `config/mpc_params.yaml` | All tunable parameters |
| `data/waypoints.csv` | Track waypoints (x, y, theta, v_ref) |
