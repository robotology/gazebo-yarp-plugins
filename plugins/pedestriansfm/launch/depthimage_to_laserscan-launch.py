import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    param_config = '/home/user1/params.yaml' #os.path.join(os.getcwd(),'param.yaml')
    return LaunchDescription([
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            remappings=[('depth', '/camera_rgbd/depth/image_rect'),
                        ('depth_camera_info', '/camera_rgbd/depth/camera_info'),
			('scan', '/depth/scan')],
            parameters=[
	        {'output_frame': 'head_link'},
		{'scan_time': 0.033},
                {'range_min': 0.45},
                {'range_max': 6.0},
                {'scan_height': 1}
	    ])
        ])
