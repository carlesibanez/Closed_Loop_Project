import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf2_ros

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        self.bridge = CvBridge()

        # ROS 2 Publishers & Broadcasters
        self.pose_pub = self.create_publisher(PoseStamped, '/vision/plate_target_pose', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None

        # ArUco Setup (4x4 dictionary, 40mm marker size)
        self.marker_size = 0.04 
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo, 
            '/wrist_camera/camera_info', 
            self.info_callback, 
            10
        )
        
        self.image_sub = self.create_subscription(
            Image, 
            '/wrist_camera/image', 
            self.image_callback, 
            10
        )
        
        self.get_logger().info("Node initialized. ArUco detection will start once camera info is received.")

    def info_callback(self, msg):
        if self.camera_matrix is None:
            try:
                self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
                if len(msg.d) == 0:
                    self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)
                    self.get_logger().warn("Gazebo published empty distortion array. Defaulting to zeros.")
                else:
                    self.dist_coeffs = np.array(msg.d, dtype=np.float64)
                    
                self.get_logger().info("Camera Intrinsics Successfully Loaded!")
            except Exception as e:
                self.get_logger().error(f"Error parsing CameraInfo: {e}")

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        try:
            # 1. Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # rectified_image = cv2.undistort(cv_image, self.camera_matrix, self.dist_coeffs)
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # corners, ids, rejected = self.detector.detectMarkers(gray_image)
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray_image, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None and len(ids) > 0:
                # Estimate Pose
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
                
                # Draw a green bounding box around the marker
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec[0], tvec[0], 0.02)

                self.publish_pose_and_tf(msg.header, rvec[0][0], tvec[0][0])

            # 2. Display the image
            cv2.imshow("Step 4: Aruco Tracker", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"CV Error during processing: {e}")

    def publish_pose_and_tf(self, header, rvec, tvec):

        header.frame_id = 'camera_link_optical'
        # A. Convert OpenCV Rotation Vector to Quaternion
        rmat, _ = cv2.Rodrigues(rvec)
        r = R.from_matrix(rmat)
        quat = r.as_quat() # Format is [x, y, z, w]
        
        # B. Publish PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.pose.position.x = tvec[0]
        pose_msg.pose.position.y = tvec[1]
        pose_msg.pose.position.z = tvec[2]
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        self.pose_pub.publish(pose_msg)
        
        # C. Broadcast TF2 Frame
        t = TransformStamped()
        t.header = header
        t.child_frame_id = 'plate_aruco_target'
        t.transform.translation.x = tvec[0]
        t.transform.translation.y = tvec[1]
        t.transform.translation.z = tvec[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()