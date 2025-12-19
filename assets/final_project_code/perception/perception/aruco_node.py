#!/usr/bin/env python3
import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data


import numpy as np
import cv2
import math


from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from ros2_aruco_interfaces.msg import ArucoMarkers
from sensor_msgs.msg import Image, CameraInfo


from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from rcl_interfaces.msg import ParameterDescriptor, ParameterType




# ---------------- Quaternion from Rotation ---------------- #
def quaternion_from_matrix(matrix):
   q = np.empty((4,), dtype=np.float64)
   M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
   t = np.trace(M)
   if t > M[3, 3]:
       q[3] = t
       q[2] = M[1, 0] - M[0, 1]
       q[1] = M[0, 2] - M[2, 0]
       q[0] = M[2, 1] - M[1, 2]
   else:
       i, j, k = 0, 1, 2
       if M[1, 1] > M[0, 0]:
           i, j, k = 1, 2, 0
       if M[2, 2] > M[i, i]:
           i, j, k = 2, 0, 1
       t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
       q[i] = t
       q[j] = M[i, j] + M[j, i]
       q[k] = M[k, i] + M[i, k]
       q[3] = M[k, j] - M[j, k]
   q *= 0.5 / math.sqrt(t * M[3, 3])
   return q




# =================== MAIN NODE =================== #
class ArucoNode(rclpy.node.Node):
   def __init__(self):
       super().__init__("aruco_node")


       # ------------ Declare parameters ------------- #
       self.declare_parameter(
           name="aruco_dictionary_id",
           value="DICT_5X5_250",
           descriptor=ParameterDescriptor(
               type=ParameterType.PARAMETER_STRING,
               description="Aruco Dictionary used to generate markers.",
           ),
       )


       self.declare_parameter(
           name="image_topic",
           value="/camera/color/image_raw",
           descriptor=ParameterDescriptor(
               type=ParameterType.PARAMETER_STRING,
               description="Image topic to subscribe to.",
           ),
       )


       self.declare_parameter(
           name="camera_info_topic",
           value="/camera/color/camera_info",
           descriptor=ParameterDescriptor(
               type=ParameterType.PARAMETER_STRING,
               description="Camera info topic.",
           ),
       )


       self.declare_parameter(
           name="camera_frame",
           value="camera_color_optical_frame",
           descriptor=ParameterDescriptor(
               type=ParameterType.PARAMETER_STRING,
               description="Camera frame to use for TF publication.",
           ),
       )


       # ----------- Apply parameter values ----------- #
       dict_name = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
       self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
       self.info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
       self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value


       # ----------- HARD-CODED MARKER SIZE MAP ---------- #
       #  Marker 1 = TurtleBot marker (50 mm)
       #  Marker 2 = Cube marker (25 mm)
       #
       self.marker_size_map = {
           1: 0.025,     # Cube marker
           3: 0.05    # TurtleBot marker


       }
       self.get_logger().info(f"📌 Hard-coded marker size map: {self.marker_size_map}")


       # -------- Load dictionary safely -------- #
       try:
           dictionary_id = cv2.aruco.__getattribute__(dict_name)
       except AttributeError:
           self.get_logger().error(f"❌ Invalid aruco_dictionary_id: {dict_name}")
           return


       self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
       self.aruco_parameters = cv2.aruco.DetectorParameters_create()
       self.aruco_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX


       # -------- Subscriptions & Publishers ---------- #
       self.bridge = CvBridge()
       self.info_msg = None
       self.intrinsic_mat = None
       self.distortion = None


       self.info_sub = self.create_subscription(CameraInfo, self.info_topic, self.info_callback, qos_profile_sensor_data)
       self.create_subscription(Image, self.image_topic, self.image_callback, qos_profile_sensor_data)


       self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
       self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)
       self.tf_broadcaster = TransformBroadcaster(self)


       self.get_logger().info("🎉 Aruco Node Started")




   # ================= CAMERA INFO ================= #
   def info_callback(self, info_msg):
       self.info_msg = info_msg
       self.intrinsic_mat = np.array(info_msg.k).reshape(3, 3)
       self.distortion = np.array(info_msg.d)
       self.get_logger().info("📷 Camera intrinsics loaded.")
       self.destroy_subscription(self.info_sub)




   # ================= IMAGE CALLBACK =============== #
   def image_callback(self, img_msg):
       if self.info_msg is None:
           self.get_logger().warn("⚠️ No camera info received yet.")
           return


       frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
       gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


       corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dictionary, parameters=self.aruco_parameters)


       if ids is None:
           return


       detections = [(int(ids[i][0]), corners[i]) for i in range(len(ids))]


       # 🔵 PRINT ALL DETECTED MARKER IDs
       detected_ids = [d[0] for d in detections]
       self.get_logger().info(f"🔍 Detected markers: {detected_ids}")


       pose_array = PoseArray()
       pose_array.header.stamp = img_msg.header.stamp
       pose_array.header.frame_id = self.camera_frame


       markers_msg = ArucoMarkers()
       markers_msg.header = pose_array.header


       # ---- Estimate pose for markers 1 & 2 ONLY ---- #
       for m_id, m_corners in detections:


           # Only allow marker 1 (robot) and marker 2 (cube)
           if m_id not in self.marker_size_map:
               self.get_logger().warn(f"🚫 Ignoring unknown marker id {m_id}")
               continue


           size = self.marker_size_map[m_id]


           rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
               [m_corners], size, self.intrinsic_mat, self.distortion
           )


           rvec = rvec[0][0]
           tvec = tvec[0][0]


           pose = Pose()
           pose.position.x = tvec[0]
           pose.position.y = tvec[1]
           pose.position.z = tvec[2]


           rot_matrix = np.eye(4)
           rot_matrix[:3, :3] = cv2.Rodrigues(rvec)[0]
           quat = quaternion_from_matrix(rot_matrix)


           pose.orientation.x = quat[0]
           pose.orientation.y = quat[1]
           pose.orientation.z = quat[2]
           pose.orientation.w = quat[3]


           pose_array.poses.append(pose)
           markers_msg.marker_ids.append(m_id)
           markers_msg.poses.append(pose)


           # Publish TF
           tf_msg = TransformStamped()
           tf_msg.header = markers_msg.header
           tf_msg.child_frame_id = f"ar_marker_{m_id}"


           tf_msg.transform.translation.x = tvec[0]
           tf_msg.transform.translation.y = tvec[1]
           tf_msg.transform.translation.z = tvec[2]


           tf_msg.transform.rotation.x = quat[0]
           tf_msg.transform.rotation.y = quat[1]
           tf_msg.transform.rotation.z = quat[2]
           tf_msg.transform.rotation.w = quat[3]


           self.tf_broadcaster.sendTransform(tf_msg)


       # Publish results
       self.poses_pub.publish(pose_array)
       self.markers_pub.publish(markers_msg)


       cv2.aruco.drawDetectedMarkers(frame, corners, ids)
       cv2.imshow("Aruco Detection", frame)
       cv2.waitKey(1)




# ================= ENTRY POINT ================= #
def main():
   rclpy.init()
   node = ArucoNode()
   rclpy.spin(node)
   node.destroy_node()
   cv2.destroyAllWindows()
   rclpy.shutdown()




if __name__ == "__main__":
   main()
