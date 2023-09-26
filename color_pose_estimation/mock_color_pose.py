import rclpy
from rclpy.node import Node
from color_pose_msgs.msg import ColorPoseArray
from color_pose_msgs.msg import ColorPose
from std_msgs.msg import Header
from geometry_msgs.msg import Pose


class MockColor(Node):

    def __init__(self):
        super().__init__('mock_publisher')
        self.publisher = self.create_publisher(ColorPoseArray, 'color_pose_estimation/color_pose_array', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = ColorPose()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = -0.216
        msg.pose.position.y = -0.04
        msg.pose.position.z = -1.065
        msg.pose.orientation.x=0.707
        msg.pose.orientation.y=0.707
        msg.pose.orientation.z=0.0
        msg.pose.orientation.w=0.001
        msg.color = "red"
        msg.element = "cube"

        msg2 = ColorPose()
        msg2.header.stamp = self.get_clock().now().to_msg()
        msg2.header.frame_id = "world"
        msg2.pose.position.x = -0.117
        msg2.pose.position.y = 0.046
        msg2.pose.position.z = 1.059
        msg2.pose.orientation.x=0.707
        msg2.pose.orientation.y=0.707
        msg2.pose.orientation.z=0.0
        msg2.pose.orientation.w=0.001
        msg2.color = "red"
        msg2.element = "cube_holder"

        msg3 = ColorPose()
        msg3.header.stamp = self.get_clock().now().to_msg()
        msg3.header.frame_id = "world"
        msg3.pose.position.x = -0.216
        msg3.pose.position.y = 0.126
        msg3.pose.position.z = -1.065
        msg3.pose.orientation.x=0.707
        msg3.pose.orientation.y=0.707
        msg3.pose.orientation.z=0.0
        msg3.pose.orientation.w=0.001
        msg3.color = "blue"
        msg3.element = "cube"

        msg4 = ColorPose()
        msg4.header.stamp = self.get_clock().now().to_msg()
        msg4.header.frame_id = "world"
        msg4.pose.position.x = -0.216
        msg4.pose.position.y = -0.04
        msg4.pose.position.z = -1.065
        msg4.pose.orientation.x=0.707
        msg4.pose.orientation.y=0.707
        msg4.pose.orientation.z=0.0
        msg4.pose.orientation.w=0.001
        msg4.color = "blue"
        msg4.element = "cube_holder"

        msg5 = ColorPose()
        msg5.header.stamp = self.get_clock().now().to_msg()
        msg5.header.frame_id = "world"
        msg5.pose.position.x = -0.312
        msg5.pose.position.y = 0.043
        msg5.pose.position.z = 1.067
        msg5.pose.orientation.x=0.707
        msg5.pose.orientation.y=0.707
        msg5.pose.orientation.z=0.0
        msg5.pose.orientation.w=0.001
        msg5.color = "green"
        msg5.element = "cube"

        msg6 = ColorPose()
        msg6.header.stamp = self.get_clock().now().to_msg()
        msg6.header.frame_id = "world"
        msg6.pose.position.x = -0.216
        msg6.pose.position.y = 0.126
        msg6.pose.position.z = -1.065
        msg6.pose.orientation.x=0.707
        msg6.pose.orientation.y=0.707
        msg6.pose.orientation.z=0.0
        msg6.pose.orientation.w=0.001
        msg6.color = "green"
        msg6.element = "cube_holder"
        print("here")
        print(msg6)


        msg_array = ColorPoseArray()
        msg_array.header.frame_id = "world"
        msg_array.color_poses.append(msg)
        msg_array.color_poses.append(msg2)
        msg_array.color_poses.append(msg3)
        msg_array.color_poses.append(msg4)
        msg_array.color_poses.append(msg5)
        msg_array.color_poses.append(msg6)

        self.publisher.publish(msg_array)
        self.get_logger().info('Publishing color pose')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    mock_color = MockColor()

    rclpy.spin(mock_color)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mock_color.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()