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
    
  ign_gazebo = IncludeLaunchDescription( 
    PythonLaunchDescriptionSource(
        os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')
        ),
  )
  spawn_sdf = Node(package='ros_ign_gazebo', executable='create',
			arguments=['-name', 'rm2',
				'-file', os.path.join(pkg, 'models', 'rm2_module', 'model.sdf')],
			output='screen')

  return LaunchDescription([
    DeclareLaunchArgument(
		  'ign_args', default_value=[os.path.join(pkg, 'worlds', 'cave_world.sdf') +
					 ' -v 2 --gui-config ' +
					 os.path.join(pkg, 'ign', 'gui.config'), ''],
		  description='Ignition Gazebo arguments'),
        ign_gazebo,
        spawn_sdf,
    ])