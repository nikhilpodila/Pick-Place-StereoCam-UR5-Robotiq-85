# Pick-Place-StereoCam-UR5-Robotiq-85
Done by Nikhil Podila

Pick and Place + Obstacle Avoidance task on a UR5 arm + Robotiq 85 gripper using an external stereocamera

## Installation
To install, clone this repository or each folder in the repository to your catkin workspace and run `catkin_make`.<br>
Standard `ROS` packages including `Rviz`, `Gazebo`, `Moveit` and `Octomap` are used.

## Running the program
To run the experiment, execute the following command:
```
roslaunch ur5_pickplace_task pick_place_task.launch
```  
On running the command, `Rviz` and `Gazebo` windows are launched and the task is initiated.

## Screen Recording
The file `Screen Recording Pick Place Task.mov` in this repository shows the experiment in progress.

## External packages
The `ur5` and `robotiq` packages in this repository are modified versions of [this repository](https://github.com/utecrobotics/ur5) and its dependencies.
