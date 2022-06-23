import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg = get_package_share_directory('rm2_simulation')
    urdf_path = pkg + '/models/rm2/rm2.urdf'

    
    ign_gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
    )

    spawn = Node(
                  package='ros_ign_gazebo', executable='create',
                  arguments=[
                    '-name', 'rm2_sim',
                    '-file',  os.path.join(pkg, 'models', 'rm2', 'rm2_sim', 'model.sdf'),
                    '-z', '-0.18',
                    '-y', '-0.65',
                    '-x', '3.73',
                    ],
                output='screen',
                )
    
    bridge = Node(
      package='ros_ign_bridge',
      executable='parameter_bridge',
      arguments=[
      # Mappings (IGN -> ROS2)
      # The ROS message type is followed by an @, [, or ] symbol where:

      # @ is a bidirectional bridge.
      # [ is a bridge from Ignition to ROS.
      # ] is a bridge from ROS to Ignition.
      # https://github.com/ignitionrobotics/ros_ign/tree/ros2/ros_ign_bridge

      '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',                    
             
                ],
      remappings=[
            ('/cmd_vel', 'cmd_vel'),
        ],
        output='screen')


    state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
				output='screen',
				parameters = [
					{'ignore_timestamp': False},
					{'use_tf_static': True},
					{'robot_description': open(urdf_path).read()}],
				arguments = [urdf_path])

    rviz_config_file = LaunchConfiguration('rviz_config',
						default=os.path.join(pkg, 'rviz', 'urdf_config.rviz'))

    rviz2 = Node(package='rviz2', executable='rviz2',
					name='rviz2',
					arguments=['-d', rviz_config_file],
					output='screen',)
    
    return LaunchDescription([
      DeclareLaunchArgument(
          'ign_args',
          default_value=[os.path.join(pkg, 'worlds', 'cave_world.sdf')]),
        ign_gazebo,
        spawn,
        bridge,
        # state_publisher,
        # rviz2
    ])

