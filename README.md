# G1 Simulation

## Running the Simulation

To start the simulation environment, run the following script:

```bash
./start_simulation.sh
```

This will run two docker containers in the background:
- `g1_sim_isaaclab`: Runs the 3D physics simulation in NVIDIA Isaac Lab, including scene rendering and sensor simulation.
- `g1_sim_controller`: Runs the control scripts for the G1 robot that communicate with the simulation suite.

## Stopping the Simulation

To stop the simulation and clean up the running docker containers, simply run:

```bash
./stop_simulation.sh
```

## Mapping and Localization

The controller container uses `slam_toolbox` for SLAM and supports both mapping and localization modes, determined by the `SLAM_MODE` environment variable.

### Mapping
To create a map of a new environment:
1. Run the `g1_sim_controller` container with `SLAM_MODE=mapping`. (You can add `-e SLAM_MODE=mapping` to the `docker run` command in `g1_sim_controller/run.sh`).
2. Move the robot around the scene to build the map.
3. Save the map to disk by running the `save_map_local.sh` (or `save_map.sh`) script. This will export the `.posegraph`, `.data`, `.pgm`, and `.yaml` files.

### Localization
By default, the simulation starts in `localization` mode (`SLAM_MODE=localization`).
Once a map is generated and placed in the appropriate directory, the robot will load it and continuously localize itself against the map automatically.

## Node Startup Behavior

When you start the `g1_sim_controller` ROS 2 package, there are built-in dependencies that must be met before various nodes fully launch:
1. **Simulation Synchronization**: The package waits until it detects messages on the `sim_state` topic via DDS. It will not launch the main ROS 2 nodes until the simulation is actively publishing its state.
2. **Nav2 and Lidar**: The Nav2 stack is **only** enabled when starting in `localization` mode. When loading, Nav2 will wait until the lidar `/scan` topic begins publishing valid data before becoming fully active.

## Network Configuration

The simulation components communicate using DDS and ROS 2. 
- **ROS_DOMAIN_ID**: `2`
- **DDS Domain**: `2`
- **Network Interface**: `eno1`

> [!IMPORTANT]
> If you are using the InOrbit agent, make sure to update your `~/.inorbit/local/agent.env.sh` file so that its `ROS_DOMAIN_ID` matches the network configuration used here (e.g., `export ROS_DOMAIN_ID=2`).
