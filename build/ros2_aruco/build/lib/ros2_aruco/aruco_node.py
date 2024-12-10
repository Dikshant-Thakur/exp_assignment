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
	    #Register the shutdown hook            
        #self.on_shutdown(self.stop_robot)
	            

        # Subscriptions and Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data)  
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.compressed_callback, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.move_callback, qos_profile_sensor_data)

        # Camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        # Aruco detector settings
        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
  
   
    def move_callback(self, msg):
        print(f"Received msg: {msg}")
        new_msg = Twist()
        new_msg.angular.z = 0.5
        self.cmd_vel_pub.publish(new_msg)

   
    def compressed_callback(self, compressed_data):
        np_arr = np.frombuffer(compressed_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        (rows, cols, channels) = image_np.shape
        vel = Twist()
        vel.angular.z = 0.5
        self.cmd_vel_pub.publish(vel)
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(image_np, self.aruco_dictionary, parameters=self.aruco_parameters)
        if marker_ids is not None:
            for i, marker_id in enumerate(marker_ids.flatten()):
                if 11 <= marker_id <= 15:
                    cv2.polylines(image_np, [np.int32(corners[i])], True, (0, 255, 0), 2)
                    cv2.putText(image_np, f"ID: {marker_id}", 
                                (int(corners[i][0][0][0]), int(corners[i][0][0][1] - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.imshow(f"ArUco Marker ID {marker_id}", image_np)
                    cv2.waitKey(1)

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
 
