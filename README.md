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
**Note:** You should **not** close the terminal running the Gazebo simulation (`gazebo_aruco.launch.py`). The simulation must continue running while you launch one of the above nodes in a different terminal.

## How It Works

- The robot starts in the center of the markers placed in a circle.
- The system will rotate the robot to find the ArUco markers in a circular pattern.
- Each time a marker is found, an image with a circle around the marker will be published..

## Important Notes

- When you run and stop any Python script (e.g., using `ctrl + c`), the node stops, but the action continues. For example:
  - If you kill the `aruco_node` (which controls the robot), the robot will keep rotating in the simulation. To stop this, run the following command in a separate terminal:
    ```bash
    ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.0}}"
    ```
  - Similarly, if you stop the camera node, the camera will continue moving. To stop the camera, run the following command in a separate terminal:
    ```bash
    ros2 topic pub /camera_joint_controller/commands std_msgs/Float64MultiArray "{data: [0.0]}"
    ```



