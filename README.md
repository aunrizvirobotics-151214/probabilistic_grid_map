# probabilistic_grid_map
This project implements occupancy grid mapping to build a 2D map of the environment using noisy sensor data.

The approach uses probabilistic updates to estimate cell occupancy and incrementally constructs the map as the robot explores the environment.


<img width="1920" height="1080" alt="Screenshot from 2026-03-31 17-33-07" src="https://github.com/user-attachments/assets/1e9aa243-27d7-4a23-a82f-71783580fe60" />

<img width="1920" height="1080" alt="Screenshot from 2026-03-31 17-32-59" src="https://github.com/user-attachments/assets/993b2c73-1bed-470a-aa3e-5c77baef2ed3" />

<img width="1920" height="1080" alt="Screenshot from 2026-03-31 17-32-56" src="https://github.com/user-attachments/assets/b8e1550f-1c83-4763-801d-47e9f8cc829a" />

<img width="1920" height="1080" alt="Screenshot from 2026-03-31 17-30-57" src="https://github.com/user-attachments/assets/55fccdb9-c4f1-44a8-b943-cc703714a7c7" />


## Simulation Setup
All algorithms are implemented and tested using:
- ROS2 (Humble)
- Gazebo
- ArticubotOne robot (based on John Evan’s tutorial)
