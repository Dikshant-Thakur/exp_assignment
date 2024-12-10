import sys
import time
import numpy as np
import cv2
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from ros2_aruco import transformations
from std_msgs.msg import String
from scipy.ndimage import filters

VERBOSE = True

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')
        # Initialize parameters
        self.declare_parameter("marker_size", .0625)
        self.declare_parameter("aruco_dictionary_id", "DICT_ARUCO_ORIGINAL")
        self.declare_parameter("compressed_image_topic", "/output/image_raw/compressed")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_frame", None)

        # Fetch parameter values
        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        compressed_image = self.get_parameter("compressed_image_topic").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

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

        # Subscriptions and Publishers

        #self.image_pub = self.create_publisher(CompressedImage, compressed_image, 10)  #NEW ONE
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data)  

        self.create_subscription(Image, image_topic, self.callback, qos_profile_sensor_data)  #NEW ONE
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.compressed_callback, qos_profile_sensor_data)  #NEW ONE



        self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data) 
        self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)          
        self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)   
        

        # Camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None
        # Aruco detector settings
        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)   #aruco_dictionary - Yeh ek list hai jisme ArUco markers ke patterns hote hain. Yeh batata hai ki kaunsa marker kaunsa pattern represent karta hai.
        self.aruco_parameters = cv2.aruco.DetectorParameters_create() #Parameters ka matlab hai settings ya rules jo marker detection ke liye use hote hain. Yeh ek object hai jo marker ko accurately detect karne ke liye settings ko define karta hai.
        self.bridge = CvBridge()  

    def compressed_callback(self, compressed_data):
        np_arr = np.frombuffer(compressed_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        (rows, cols, channels) = image_np.shape
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(image_np, self.aruco_dictionary, parameters=self.aruco_parameters)
        if marker_ids is not None:
            for i, marker_id in enumerate(marker_ids.flatten()):
                if 11 <= marker_id <= 15:
                    cv2.polylines(image_np, [np.int32(corners[i])], True, (0, 255, 0), 2) # Create green color boundary
                    cv2.putText(image_np, f"ID: {marker_id}", (int(corners[i][0][0][0]), int(corners[i][0][0][1] - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)  #ID number shown
                    cv2.imshow(f"ArUco Marker ID {marker_id}", image_np)
                    cv2.waitKey(1)
        #if cols > 60 and rows > 60:
            #cv2.circle(image_np, (50, 50), 50, 255)
            #cv2.circle(image_np, (30, 30), 20, (0, 255, 0), 5)
        
        #cv2.imshow('window', image_np)
        #cv2.waitKey(2)

        #msg = CompressedImage()
        #.header.stamp = node.get_clock().now()
        #msg.format = "jpeg"
        #msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tobytes()
        #self.image_pub.publish(msg)


    
    def info_callback(self, info_msg): # camera ki calibration values ko store kar raha hai, jo future image processing aur ArUco marker detection mein kaam aayengi.
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(info_msg.k), (3, 3))
        self.distortion = np.array(info_msg.d)
        self.destroy_subscription(self.info_sub)
    
    def image_callback(self, img_msg):
        if not self.info_msg:
            self.get_logger().warn("No camera info received yet!")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, 'mono8')
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image message: {e}")
            return

        # Aruco detection and pose estimation
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)
        if marker_ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.intrinsic_mat, self.distortion)
            pose_array = PoseArray()
            markers = ArucoMarkers()

            for i, marker_id in enumerate(marker_ids.flatten()):
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = tvecs[i][0]

                rot_matrix = np.eye(4)
                rot_matrix[:3, :3] = cv2.Rodrigues(rvecs[i][0])[0]
                quat = transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
                pose_array.poses.append(pose)
                markers.marker_ids.append(marker_id)

            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)  
            
        
def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

