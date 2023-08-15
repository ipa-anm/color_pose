import sys
import os
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import sensor_msgs.msg as sensor_msgs
import color_pose_estimation.registration as reg
import color_pose_estimation.detect_color as cdet
import color_pose_estimation.detect_color_scene as cdet_s
import color_pose_msgs.msg
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np
import open3d as o3d
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, Pose, TransformStamped, PoseStamped
import tf2_ros
from tf2_ros import TransformException, TransformBroadcaster, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf2_geometry_msgs
import image_geometry
import ctypes
import math
import struct
import pyrealsense2 as rs2
import time
from tf2_geometry_msgs import do_transform_pose
from rclpy.executors import MultiThreadedExecutor


# COLORTYPES

GREEN = 1
BLUE = 2
RED = 3
YELLOW = 4


class Color_Pose_Estimation(Node):
    def __init__(self):
        super().__init__('color_estimation_node')
        self.br = CvBridge()
        self.center = [0.0, 0.0, 0.0]
        self.tfBuffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=1))
        self.listener = tf2_ros.TransformListener(
            self.tfBuffer, self, spin_thread=True)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.image_sub = Subscriber(
            self, sensor_msgs.Image, "/camera/color/image_raw", qos_profile=qos_profile_sensor_data)
        self.aligned_depth_sub = Subscriber(
            self, sensor_msgs.Image, "/camera/aligned_depth_to_color/image_raw", qos_profile=qos_profile_sensor_data)
        self.camera_info_sub = Subscriber(
            self, sensor_msgs.CameraInfo, "/camera/aligned_depth_to_color/camera_info")
        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.aligned_depth_sub, self.camera_info_sub], 10, 0.1,)
        self.camera_frame = "camera_depth_optical_frame"
        self.box_frame = "box_frame"
        self.ts.registerCallback(self.color_estimation_callback)
        self.publisher_color_image = self.create_publisher(
            sensor_msgs.Image, 'color_pose_estimation/color_image', 10)
        # self.publisher = self.create_publisher(tf2_geometry_msgs.PoseStamped, "/pose_topic", 10)
        self.publisher_color_pose = self.create_publisher(
            color_pose_msgs.msg.ColorPose, '/color_pose_estimation/color_pose', 10)
        # self.publisher_color_pose_array = self.create_publisher(
        #     color_pose_msgs.msg.ColorPoseArray, '/color_pose_estimation/color_pose_array', 10)

    def color_estimation_callback(self, image, depth, camera_info):
        ###### Image Processing#########

        # CV2 bridge for RGB image in CV2 format
        self.current_frame = self.br.imgmsg_to_cv2(
            image, desired_encoding="bgr8")
        #print(self.current_frame.shape)

        # cv2.imshow("image", self.current_frame)
        # key = cv2.waitKey(1)

       # CV2 bridge for depth image in CV2 format
        depth_image = self.br.imgmsg_to_cv2(depth, "passthrough")
        depth_array = np.array(depth_image, dtype=np.float32)

        # create Open3d Image from CV2
        o3d_depth = o3d.geometry.Image(depth_array)
        o3d_rgb = o3d.geometry.Image(self.current_frame)

        # combine both Images to RGBDImage
        try:
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d_rgb, o3d_depth)
        except RuntimeError as e:
            print(f"Exception caught: {str(e)}")
            return

       # set camera intrinsics
        _intrinsics = rs2.intrinsics()
        _intrinsics.width = camera_info.width
        _intrinsics.height = camera_info.height
        _intrinsics.ppx = camera_info.k[2]
        _intrinsics.ppy = camera_info.k[5]
        _intrinsics.fx = camera_info.k[0]
        _intrinsics.fy = camera_info.k[4]
        # _intrinsics.model = cameraInfo.distortion_model
        _intrinsics.model = rs2.distortion.brown_conrady
        _intrinsics.coeffs = [i for i in camera_info.d]

        intrinsic_o3d = o3d.camera.PinholeCameraIntrinsic(
            width=_intrinsics.width, height=_intrinsics.height, fx=_intrinsics.fx, fy=_intrinsics.fy, cx=_intrinsics.ppx, cy=_intrinsics.ppy)

        # create Pointcloud from RGBDImage
        self.o3d_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, intrinsic=intrinsic_o3d)

        if o3d.geometry.PointCloud.is_empty(self.o3d_pcd):
            return

        # color detection node returns the bounding box of the found object in
        # the bounding box format (x,y,w,h)
        color_array, img = cdet_s.detect(self.current_frame)
        image_message = self.br.cv2_to_imgmsg(img, encoding="passthrough")
        self.publisher_color_image.publish(image_message)
        self.get_logger().info('Publishing a color image ')

        for color, rectangles in color_array.items():
            print(f'Color: {color}')
            is_box = True
            for rect in rectangles:
                # center of pointcloud from edges of bounding box
                center_image_x = int(rect[0]+(rect[2]/2))
                center_image_y = int(rect[1]+(rect[3]/2))

                # get corresponding depth values from depth map with pixel from RGB
                depth_1 = depth_array[center_image_y, center_image_x]*0.001
                depth_2 = depth_array[rect[1], rect[0]]*0.001

                # self.camera_model = image_geometry.PinholeCameraModel()
                # self.camera_model.fromCameraInfo(camera_info)

                center_point = self.convert_pixel_to_point(
                    center_image_x, center_image_y, depth_1, camera_info, _intrinsics)
                corner_point = self.convert_pixel_to_point(
                    rect[0], rect[1], depth_2, camera_info, _intrinsics)

                size_y = (center_point[0]-corner_point[0])*3
                size_x = (center_point[1]-corner_point[1])*3

                # print(size_y, size_x)
                size = np.array([size_y, size_x, 0.4])

                # Define bounding box of object in Poincloud Coordinate system
                center = np.array(
                    [center_point[0], center_point[1], center_point[2]])

                r = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
                bbox = o3d.geometry.OrientedBoundingBox(center, r, size)

                # visualize the bounding box and crop the pointcloud around the bounding box coordinates
                # o3d.visualization.draw_geometries([self.o3d_pcd, bbox])
                try:
                    self.o3d_pcd = self.o3d_pcd.crop(bbox)
                except Exception as e:
                    self.get_logger().error("Exception occurred: {0}".format(e))

                # o3d.io.write_point_cloud("copy_of_fragment.ply", self.o3d_pcd)
                if is_box:
                    self.transform_pose(center, color)
                    is_box = False
                else:
                    self.transform_pose(center, f'{color}_holder')


    def convert_pixel_to_point(self, x, y, depth, cameraInfo, _intrinsics):
        result = rs2.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)
        return result

    def transform_pose(self, center, color = "Red"):

        try:
            # Look up the transform from "frame1" to "frame2"
            transform = self.tfBuffer.lookup_transform(
                "camera_depth_optical_frame", "world", rclpy.time.Time(), timeout=rclpy.time.Duration(seconds=1))
        except Exception as e:
            self.get_logger().error("Exception occurred: {0}".format(e))

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_depth_optical_frame"
        t.child_frame_id = str(color)
        t.transform.translation.x = center[0]-0.0148 # https://github.com/IntelRealSense/realsense-ros#---extrinsics-from-sensor-a-to-sensor-b
        t.transform.translation.y = center[1]
        t.transform.translation.z = center[2]
        t.transform.rotation.w = 1.0

        self.tf_static_broadcaster.sendTransform(t)

        transformed_pose = None
        try:
            transformed_pose = self.tfBuffer.lookup_transform("world", str(color), rclpy.time.Time(), timeout=rclpy.time.Duration(seconds=1))
        except Exception as e:
            self.get_logger().error("Exception occurred: {0}".format(e))
            return

        color_pose = color_pose_msgs.msg.ColorPose()
        color_pose.header.frame_id = transformed_pose.header.frame_id
        color_pose.pose.position.x = transformed_pose.transform.translation.x
        color_pose.pose.position.y = transformed_pose.transform.translation.y
        color_pose.pose.position.z = transformed_pose.transform.translation.z
        color_pose.pose.orientation.w = 1.0
        color_pose.color = str(color)

        # Publish the transformed pose
        self.publisher_color_pose.publish(color_pose)


def main(args=None):
    rclpy.init(args=args)
    cpe = Color_Pose_Estimation()
    rclpy.spin(cpe)
    cpe.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
