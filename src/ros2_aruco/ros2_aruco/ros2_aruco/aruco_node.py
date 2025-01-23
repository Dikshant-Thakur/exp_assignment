import sys
import time
import numpy as np
import cv2
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from geometry_msgs.msg import PoseArray, Pose, Twist
from ros2_aruco_interfaces.msg import ArucoMarkers
from ros2_aruco import transformations
from std_msgs.msg import String
from scipy.ndimage import filters
import signal

VERBOSE = True

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')
        
        # Initialize parameters
        self.declare_parameter("aruco_dictionary_id", "DICT_ARUCO_ORIGINAL")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")

        # Fetch parameter values
        dictionary_id_name = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value   
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value

        # Set up aruco dictionary
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if not isinstance(dictionary_id, int):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(f"Invalid aruco_dictionary_id: {dictionary_id_name}")
            valid_options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error(f"Valid options: {valid_options}")
            return
	            
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Subscribers
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data)  
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.compressed_callback, qos_profile_sensor_data)

        # Camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        # Aruco detector settings
        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()

        # Create ROS2 Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Aruco node initialized')

        # Variables for state process execution
        self.state = 'start'
        self.found_marker_ids = []
        self.search_marker_ids = []

        self.marker_index = 0
        self.prev_marker_index = -1
        
        self.search_ended = False

        # Initialize the signal handler for SIGINT (Ctrl+C)
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signum, frame):
        self.get_logger().info('Ctrl+C pressed, stopping robot.')
        msg = Twist()
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        sys.exit(0)

    def timer_callback(self):
        if self.state == 'start':
            msg = Twist()
            msg.angular.z = 0.7
            self.cmd_vel_pub.publish(msg)
            print("Inspection started: Waiting to detect the 5 markers")
            self.state = 'inspect'

        elif self.state == 'inspect':
            if len(self.found_marker_ids) == 5:
                # Order the list found_marker_ids
                self.found_marker_ids.sort()
                print("Successfully detected the 5 markers. Searching for each one in sequence...")
                print(F"Markers found: {self.found_marker_ids}")
                self.state = 'search'

        elif self.state == 'search':
            if self.search_ended:
                self.state = 'done'
            # To print the marker_id only once
            elif self.marker_index != self.prev_marker_index:
                print(f"Searching for marker_id {self.found_marker_ids[self.marker_index]}")
                self.search_marker_ids.append(self.found_marker_ids[self.marker_index])
                self.prev_marker_index = self.marker_index  # Update the previous marker_index
            
        elif self.state == 'done':
            # Stop the robot
            msg = Twist()
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
            # Shut down the ROS2 node
            print("Success! Press CTRL+C to shutdown the node")
            self.state = 'shutdown'
        
        elif self.state == 'shutdown':
            pass
   
    def compressed_callback(self, compressed_data):
        np_arr = np.frombuffer(compressed_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        (rows, cols, channels) = image_np.shape
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(image_np, self.aruco_dictionary, parameters=self.aruco_parameters)

        # Check if any markers were detected
        if marker_ids is None:
            return
        
        # Sends the marker_ids to the found_marker_ids array without repeating
        if self.state == 'inspect':
            for marker_id in marker_ids.flatten():
                if marker_id not in self.found_marker_ids:
                    self.found_marker_ids.append(marker_id)
                    print(f"Marker ID {marker_id} detected")

        # Draw detected markers
        elif self.state == 'search' and self.marker_index < 5:
            for i, marker_id in enumerate(marker_ids.flatten()):
                if marker_id == self.found_marker_ids[self.marker_index]:
                    self.update_marker_index()
                if marker_id in self.search_marker_ids:
                    cv2.polylines(image_np, [np.int32(corners[i])], True, (0, 255, 0), 2)
                    cv2.putText(image_np, f"ID: {marker_id}", 
                                (int(corners[i][0][0][0]), int(corners[i][0][0][1] - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.imshow(f"Aruco Marker ID {marker_id}", image_np)
                    cv2.waitKey(1)

    def update_marker_index(self):
        if self.marker_index < 4:
            self.marker_index += 1 
        else:
            self.search_ended = True

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(info_msg.k), (3, 3))
        self.distortion = np.array(info_msg.d)
        self.destroy_subscription(self.info_sub)


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 
