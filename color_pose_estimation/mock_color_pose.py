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
        msg.color = "red"
        msg.pose = Pose()
        msg.element = "cube"
        
        msg2 = ColorPose()
        msg2.header = Header()
        msg2.header.stamp = self.get_clock().now().to_msg()
        msg2.color = "red"
        msg2.pose = Pose()
        msg2.element = "cube_holder"
        msg_array = ColorPoseArray()
        msg_array.header.frame_id = "world"
        msg_array.color_poses.append(msg)
        msg_array.color_poses.append(msg2)
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