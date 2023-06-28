

https://github.com/Joas329/rover_simulation2/assets/51135069/fe6b642d-d99b-42d6-8f83-a17ddaa0f1bd




# Rover Simulation
Repository for the code that runs the rover simulation

First of all git clone the repo to your desktop or prefered directory

## Installing dependencies
1.- Open a terminal and use `cd' to change directory to rover_simulation directory

2.- Run the dependencies installation script for installing Ros2_control Gazebo and xacro (This step assumes you have the basic ROS2 dependencies installed, if not consult Dependencies installer script from the [Main Rover Repository](https://github.com/YorkURobotics/Rover-Team-2022-3-Codebase))

```
bash src/my_bot/simulation_installer.bash
```

## Running the code

1.-In the rover_simulation directory use colcon to build the repository
```
colcon build
```
3.- source the setup script
```
source install/setup.bash
```
4.- run the launch file 
```
ros2 launch my_bot launch_sim.launch.py
```
Optional .- you can add this command to run the simulation with an obstacle world :
```
ros2 launch my_bot launch_sim.launch.py world:=./src/my_bot/worlds/obstacles.world
```
5.-Once gazebo opens open another terminal and repeat steps 2 to 4 and run the keyboard input interface command
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```
