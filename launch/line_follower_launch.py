from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch PX4 SITL in a new terminal
        ExecuteProcess(
            cmd=[
                'gnome-terminal', '--',
                'bash', '-c',
                'cd ~/PX4-Autopilot && make px4_sitl_default gazebo-classic_iris; exec bash'
            ],
            shell=False
        ),

		# Launch MicroXRCEAgent in a new terminal
		ExecuteProcess(
			cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
			name='xrce_agent',
			output='screen'
		),

        # ROS 2 controller node
        Node(
            package='line_follower_drone',
            executable='controller',
            name='controller_node',
            output='screen'
        ),

        # ROS 2 line detector node
        Node(
            package='line_follower_drone',
            executable='line_detector',
            name='line_detector_node',
            output='screen'
        ),
    ])

