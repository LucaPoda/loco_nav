# Loco Planning Project

## Setup:

Inside the loco nav Docker container, install the OMPL dependency:
```bash
sudo apt-get update && sudo apt-get install -y ros-noetic-ompl
```

## Running the Project:
1. Launch Simulation
```bash
roslaunch loco_planning_project victim_rescue.launch
```
2. Run Planner:
```bash
roslaunch loco_planning_project planner.launch algo:=prm
roslaunch loco_planning_project planner.launch algo:=acd
```
