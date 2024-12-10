# Robot Marker Detection with ArUco

This project demonstrates how to detect ArUco markers in a Gazebo simulation and control a robot to find them in order. The markers are placed in a circle, and the robot either rotates or uses the camera to detect them one by one. The ArUco markers used are from IDs 11 to 15.

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
## How It Works

- The robot starts in the center of the markers placed in a circle.
- The system will rotate the robot to find the ArUco markers in a circular pattern.
- Each time a marker is found, an image with a circle around the marker will be published on a custom topic.

## Important Notes

- When you run and stop any Python script (e.g., using `ctrl + c`), the node will stop, but the action will continue running.
  - For example, if you kill the `aruco_node` (which controls the robot), the robot will keep rotating in the simulation.
  - Similarly, if you stop the camera node, the camera will continue running even after the node is killed.


