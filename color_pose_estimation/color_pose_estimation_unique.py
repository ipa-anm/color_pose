import sys
import os
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import sensor_msgs.msg as sensor_msgs
import color_pose_estimation.registration as reg
import color_pose_estimation.detect_color as cdet
import color_pose_msgs.msg
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np
import open3d as o3d
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, Pose, TransformStamped, PoseStamped
import tf2_ros
from tf2_ros import TransformException, TransformBroadcaster, TransformBroadcaster, TransformListener
import tf2_geometry_msgs
import image_geometry
import ctypes
import math
import struct
import pyrealsense2 as rs2
import time
from tf2_geometry_msgs import do_transform_pose
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import PointCloud2, PointField

class Color_Pose_Estimation(Node):
    def __init__(self):
        super().__init__('color_pose_estimation_unique_node')
        self.br = CvBridge()
        self.center = [0.0,0.0,0.0]
        self.tfBuffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=1))
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self, spin_thread=True)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.image_sub = Subscriber(self, sensor_msgs.Image, "/camera/color/image_raw", qos_profile=qos_profile_sensor_data)
        self.aligned_depth_sub = Subscriber(self, sensor_msgs.Image, "/camera/aligned_depth_to_color/image_raw", qos_profile=qos_profile_sensor_data)
        self.camera_info_sub = Subscriber(self, sensor_msgs.CameraInfo, "/camera/aligned_depth_to_color/camera_info")
        self.ts = ApproximateTimeSynchronizer([self.image_sub, self.aligned_depth_sub, self.camera_info_sub],10,0.1,)
        self.camera_frame = "camera_depth_optical_frame"
        self.box_frame = "box_frame"      
        self.ts.registerCallback(self.color_estimation_callback)
        #self.publisher = self.create_publisher(tf2_geometry_msgs.PoseStamped, "/pose_topic", 10)
        self.publisher_ = self.create_publisher(color_pose_msgs.msg.ColorPose, '/color_pose_estimation/color_pose', 10)
        self.publisher_open3d = self.create_publisher(PointCloud2, 'o3d_result', 10)
        self.publisher_color_image = self.create_publisher(
            sensor_msgs.Image, 'color_pose_estimation/color_image', 10)

    def color_estimation_callback(self, image, depth, camera_info):
        ######Image Processing#########

        # CV2 bridge for RGB image in CV2 format
        self.current_frame = self.br.imgmsg_to_cv2(
            image, desired_encoding="bgr8")
        
        #cv2.imshow("image", self.current_frame)
        #key = cv2.waitKey(1)


       # CV2 bridge for depth image in CV2 format 
        depth_image = self.br.imgmsg_to_cv2(depth, "passthrough")
        depth_array = np.array(depth_image, dtype=np.float32)


        # create Open3d Image from CV2
        o3d_depth = o3d.geometry.Image(depth_array)
        o3d_rgb = o3d.geometry.Image(self.current_frame)


        # combine both Images to RGBDImage
        try:
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_rgb, o3d_depth)
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
        #_intrinsics.model = cameraInfo.distortion_model
        _intrinsics.model  = rs2.distortion.brown_conrady
        _intrinsics.coeffs = [i for i in camera_info.d]  

        intrinsic_o3d = o3d.camera.PinholeCameraIntrinsic(width=_intrinsics.width, height=_intrinsics.height, fx=_intrinsics.fx, fy=_intrinsics.fy, cx=_intrinsics.ppx, cy=_intrinsics.ppy)


        # create Pointcloud from RGBDImage
        self.o3d_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic=intrinsic_o3d)

        if o3d.geometry.PointCloud.is_empty(self.o3d_pcd): 
            return

        # color detection node returns the bounding box of the found object in 
        # the bounding box format (x,y,w,h)
        img, rect = cdet.detect(self.current_frame)
        image_message = self.br.cv2_to_imgmsg(img, encoding="passthrough")
        self.publisher_color_image.publish(image_message)
        if (rect[2]<10 or rect[3]<10): 
            return
        
        # center of pointcloud from edges of bounding box 
        center_image_x = int(rect[0]+(rect[2]/2))
        center_image_y = int(rect[1]+(rect[3]/2))

        # get corresponding depth values from depth map with pixel from RGB
        depth_1 = depth_array[center_image_y, center_image_x]*0.001;
        depth_2 = depth_array[rect[1], rect[0]]*0.001;



        # self.camera_model = image_geometry.PinholeCameraModel()
        # self.camera_model.fromCameraInfo(camera_info)

        center_point = self.convert_pixel_to_point(center_image_x, center_image_y, depth_1, camera_info, _intrinsics)
        corner_point = self.convert_pixel_to_point(rect[0],rect[1], depth_2, camera_info, _intrinsics)
        # ray = np.array(self.camera_model.projectPixelTo3dRay((center_image_x,center_image_y))) 
      
        # ray_z = [el / ray[2] for el in ray]  # normalize the ray so its Z-component equals 1.0
        # pt = [el * depth_1 for el in ray_z]  # multiply the ray by the depth; its Z-component should now equal the depth value
        # color_pixel = rs2.rs2_project_point_to_pixel(_intrinsics, (center_point))
        # print(color_pixel)

        size_y = (center_point[0]-corner_point[0])*5
        size_x = (center_point[1]-corner_point[1])*5

        # print(size_y, size_x)
        size = np.array([size_y, size_x,0.4])

        # Define bounding box of object in Poincloud Coordinate system 
        center = np.array([center_point[0], center_point[1], center_point[2]])

        r = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        bbox = o3d.geometry.OrientedBoundingBox(center, r, size)



        # visualize the bounding box and crop the pointcloud around the bounding box coordinates
        # o3d.visualization.draw_geometries([self.o3d_pcd, bbox])
        try: 
            self.o3d_pcd = self.o3d_pcd.crop(bbox)
        except Exception as e:
            self.get_logger().error("Exception occurred: {0}".format(e))
        
        o3d.io.write_point_cloud("copy_of_fragment.ply", self.o3d_pcd)
        
        self.center = center
        #print(self.center[0], self.center[1])

       
        # Pointcloud matching with the cropped bounding box 
        # returns a transformation matrix 4x4 consisting of 
        # the rotation and translation within the cropped area
        #print(self.o3d_pcd)

        start = time.time()
        #self.transformation_matrix, source_down, target_down, transformed_source_down = reg.register(self.o3d_pcd)
        #self.create_ros2_pointcloud2_from_open3d_pointcloud(transformed_source_down)
        
        
        end = time.time()
        print("Time for Registration in seconds")
        print(end-start)
        #time.sleep(0.25)
        # transform the received pose in Point Cloud Coordinate System to 
        # World coordinates and publish it as a color pose message
        self.transform_pose()

    

    def convert_pixel_to_point(self,x, y, depth, cameraInfo, _intrinsics):  
        result = rs2.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)
        return result
    
    def create_ros2_pointcloud2_from_open3d_pointcloud(self,pointcloud): 
        points = np.asarray(pointcloud)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        pc2_msg = PointCloud2(header=header, height=1, width=points.shape[0],
                      fields=[PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                              PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                              PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                              PointField(name="rgb", offset=16, datatype=PointField.FLOAT32, count=1)],
                      is_bigendian=False, point_step=16, row_step=16*points.shape[0], is_dense=True)
        # Fill in PointCloud2 data manually using numpy
        data = np.zeros((points.shape[0], 4), dtype=np.float32)
        data[:, 0] = points[:, 0]
        data[:, 1] = points[:, 1]
        data[:, 2] = points[:, 2]
        data[:, 3] = np.float32([255, 255, 255])  # set color to white
        pc2_msg.data = data.tobytes()

        # Publish PointCloud2 message
        self.publisher_open3d.publish(pc2_msg)


    def transform_pose(self):

        t = TransformStamped()
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.box_frame
        t.transform.translation.x = self.center[0]-0.015
        t.transform.translation.y = self.center[1]
        t.transform.translation.z = self.center[2]
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        for i in range (0,10):
            now_stamp = self.get_clock().now()
            t.header.stamp = now_stamp.to_msg()
            self.tf_broadcaster.sendTransform(t)
        try:
            # Look up the transform from "frame1" to "frame2"
            transform = self.tfBuffer.lookup_transform("camera_depth_optical_frame", "world", rclpy.time.Time(), timeout=rclpy.time.Duration(seconds=1))
            
            # Create a pose stamped message in "frame1"
            pose = PoseStamped()
            print("color_pose generated")
            pose.header.frame_id = "camera_depth_optical_frame"
            pose.pose.position.x = self.center[0]-0.015
            pose.pose.position.y = self.center[1]
            pose.pose.position.z = self.center[2]
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            
            
            # Transform the pose to "frame2"
            transformed_pose = self.tfBuffer.transform(pose, "world", timeout=rclpy.time.Duration(seconds=1))
            
            color_pose = color_pose_msgs.msg.ColorPose()
            color_pose.header.frame_id = transformed_pose.header.frame_id
            color_pose.pose.position.x = transformed_pose.pose.position.x
            color_pose.pose.position.y = transformed_pose.pose.position.y
            color_pose.pose.position.z = transformed_pose.pose.position.z
            color_pose.pose.orientation.x = 0.0
            color_pose.pose.orientation.y = 0.0
            color_pose.pose.orientation.z = 0.0
            color_pose.pose.orientation.w = 1.0
            color_pose.color = 1

            # Publish the transformed pose
            self.publisher_.publish(color_pose)
            
        except Exception as e:
            self.get_logger().error("Exception occurred: {0}".format(e))
        



def main(args=None):
    rclpy.init(args=args)
    cpe = Color_Pose_Estimation()
    rclpy.spin(cpe)
    cpe.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
