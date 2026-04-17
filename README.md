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

## Network Configuration

The simulation components communicate using DDS and ROS 2. 
- **ROS_DOMAIN_ID**: `2`
- **DDS Domain**: `2`
- **Network Interface**: `eno1`
