from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    color_pose_estimation_node = Node(
        package="color_pose_estimation",
        executable="color_pose_estimation", output='screen'
    )


    return LaunchDescription([color_pose_estimation_node])
