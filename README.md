# Robot Marker Detection with ArUco

This project demonstrates how to detect ArUco markers in a Gazebo simulation and control a robot to find them in order. The markers are placed in a circle, and the robot either rotates or uses the camera to detect them one by one. The ArUco markers used are from IDs 11 to 15.

## Requirements

To run the simulation and detection, use the following commands:

### 1. Rotating the Camera to Find Markers (ArUco marker IDs 11 to 15)
To use the camera to detect markers in a circle, run the following command:
```bash
ros2 run ros2_aruco aruco_node_camera
