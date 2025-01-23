# Robot Marker Detection with ArUco

This project demonstrates how to detect ArUco markers in a Gazebo simulation and control a robot to find them in order. The markers are placed in a circle, and the robot either rotates or uses the camera to detect them one by one. The ArUco markers used are from IDs 11 to 15.
This project uses ROS2 Foxy, so please ensure your environment is set up beforehand. 

## Requirements

To run the simulation and detection, use the following commands:

### 1. To start the Gazebo simulation:
```bash
ros2 launch robot_urdf gazebo_aruco.launch.py
```

### 2. For rotating the camera to find markers (ArUco marker IDs 11 to 15):
```bash
ros2 run ros2_aruco aruco_node_camera
```
### 3. For moving the robot to find markers (ArUco marker IDs 11 to 15);
```bash
ros2 run ros2_aruco aruco_node
```
**Note:** You should **not** close the terminal running the Gazebo simulation (`gazebo_aruco.launch.py`). The simulation must continue running while you launch one of the above nodes in a different terminal.

## How It Works

- The robot starts in the centre of the markers placed in a circle.
- The system will rotate the robot to find the ArUco markers in a circular pattern.
- Then it gives you the smallest ID among all of the IDs. 
- Then it rotates and an image with a circle around the marker will be published. 



