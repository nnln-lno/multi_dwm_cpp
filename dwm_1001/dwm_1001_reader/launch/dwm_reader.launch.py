import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    param_dir = get_package_share_directory('dwm_1001_reader')
    dwm1001_param = os.path.join(param_dir, 'config', 'dwm_config.yaml')
    
    if os.path.exists(dwm1001_param):
        print("Found file : ", dwm1001_param)
    else:
        print("File not Founded ", dwm1001_param)
        sys.exit(1)
        
    dwm1001_Node = Node(
        package='dwm_1001_reader',
        executable='dwm_reader',
        name='dwm_1001_params',
        output='screen',
        parameters=[
            dwm1001_param
        ],
    )
    
    return LaunchDescription([
        dwm1001_Node,
    ])
