from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ira_laser_tools',
            executable='laserscan_multi_merger',
            name='laserscan_multi_merger',
            parameters=[
                {"destination_frame": "base_scan"},
                {"cloud_destination_topic": "/merged_cloud"},
                {"scan_destination_topic": "/scan_multi"},
                {"laserscan_topics": "/scan /scanback"}, # LIST OF THE LASER SCAN TOPICS TO SUBSCRIBE
                {"angle_min": -3.14},
                {"angle_max": 3.14},
                {"angle_increment": 0.006},
                {"scan_time": 0.02},
                {"range_min": 0.05},
                {"range_max": 30.0},
            ]
        )
    ])
