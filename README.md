# probabilistic_grid_map
This project implements occupancy grid mapping to build a 2D map of the environment using noisy sensor data.

The approach uses probabilistic updates to estimate cell occupancy and incrementally constructs the map as the robot explores the environment.

<img width="1920" height="1080" alt="Screenshot from 2026-03-31 17-35-07" src="https://github.com/user-attachments/assets/68a80b7d-5961-46cb-b465-100aeae2f3f1" />

## Simulation Setup
All algorithms are implemented and tested using:
- ROS2 (Humble)
- Gazebo
- ArticubotOne robot (based on John Evan’s tutorial)
