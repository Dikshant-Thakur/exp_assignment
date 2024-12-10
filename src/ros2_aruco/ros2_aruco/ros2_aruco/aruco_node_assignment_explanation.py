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
        '''
        cv2.aruco.Dictionary_get(dictionary_id) Aruco markers ki ek predefined dictionary ko load karta hai, jisme markers ke unique binary patterns hote hain, aur ye dictionary marker detection aur verification ke liye use hoti hai.
        cv2.aruco.DetectorParameters_create() Aruco markers detect karne ke rules (parameters) banata hai, jaise image ko kaise scan karna hai aur markers ko kaise pehchanna hai.
        '''
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
   
    def move_callback(self, msg):
        print(f"Received msg: {msg}")
        new_msg = Twist()
        new_msg.angular.z = 0.5
        self.cmd_vel_pub.publish(new_msg)

   
    def compressed_callback(self, compressed_data):
        np_arr = np.frombuffer(compressed_data.data, np.uint8)  #Yeh line compressed_data.data ko ek byte array (uint8 type) mein convert kar rahi hai. Agar compressed_data.data ka size 100 bytes hai: np.frombuffer(compressed_data.data, np.uint8)
        #ek array banayega jiski size = 100 hogi.
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        '''
        Ye line np_arr (jo ek byte array hai) ko decode karke ek image me convert kar rahi hai, aur us image ko OpenCV format (BGR color format) me load kar rahi hai.
        cv2.imdecode  Byte array (np_arr) ko image data me decode karta hai.
        cv2.IMREAD_COLOR  Image ko color format (BGR) me load karta hai.
        '''
        #(rows, cols, channels) = image_np.shape
        vel = Twist()
        vel.angular.z = 0.5
        self.cmd_vel_pub.publish(vel)
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(image_np, self.aruco_dictionary, parameters=self.aruco_parameters) #Yeh function markers ko find karta hai aur unki positions aur IDs return karta hai.
        if marker_ids is not None:
            for i, marker_id in enumerate(marker_ids.flatten()):
                '''
                Agar marker_ids = [[5], [7], [3]] ho:
                flatten() se wo [5, 7, 3] ban jayega. => Iska matlab hai, 2D ya higher-dimensional array ko ek single line (1D array) mein badalna.
                    enumerate() se aapko milega: => Array ke current element ka value
                    i = 0, marker_id = 5
                    i = 1, marker_id = 7
                    i = 2, marker_id = 3
                '''
                if 11 <= marker_id <= 15:
                    cv2.polylines(image_np, [np.int32(corners[i])], True, (0, 255, 0), 2) #Yeh line image me ek polygon draw karne ka kaam kar rahi hai, jisme Aruco marker ke corners ko connect kiya jata hai.
                    '''
                    image_np: Yeh wo image hai jisme polygon draw hoga
                    [np.int32(corners[i])]
                        corners[i]: Yeh ith marker ke corner points ko represent karta hai.
                        np.int32(): Yeh corner points ko integer type me convert karta hai.
                        [np.int32(corners[i])]: ek list mein wrap kiya gaya hai.
                    True: Iska matlab hai polygon ko closed (connect) karna, yaani last point ko first point se connect karna.
                    (0, 255, 0): Yeh color hai jo polygon ko diya jayega (BGR format mein). Yahan green color hai.
                    2: Yeh line thickness hai, jo polygon ke edges ke liye use hoti hai.
                    '''
                    cv2.putText(image_np, f"ID: {marker_id}", 
                                (int(corners[i][0][0][0]), int(corners[i][0][0][1] - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)  #Ye marker ID text generate karega, position mention kari hai, FONT, size of the text,color, thickness. 
                    cv2.imshow(f"ArUco Marker ID {marker_id}", image_np) #Ye window display kar rha hai 
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

