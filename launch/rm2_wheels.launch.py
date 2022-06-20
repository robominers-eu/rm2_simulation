import os
import xacro
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
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    ign_gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
      #launch_arguments={'ign_args': os.path.join(pkg, 'worlds', 'empty.sdf')}.items(),
     )

    spawn = Node(
                  package='ros_ign_gazebo', executable='create',
                  arguments=[
                    '-name', 'rm2_wheels',
                    '-file',  os.path.join(pkg, 'models', 'rm2_wheels', 'model.sdf'),
                    '-z', '0.5',
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
    
    return LaunchDescription([
      DeclareLaunchArgument(
          'ign_args',
          default_value=[os.path.join(pkg, 'worlds', 'empty.sdf')]),
        ign_gazebo,
        spawn,
        bridge
    ])

