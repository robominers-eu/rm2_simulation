import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.descriptions import ParameterValue 
from launch_ros.actions import Node
use_sim_time = True

def generate_launch_description():
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg = get_package_share_directory('rm2_simulation')
    path_to_urdf = pkg+'/urdf/rm2.urdf.xacro'

    
    ign_gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
    )

    state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
        'robot_description': ParameterValue(Command(['xacro ', str(path_to_urdf)]),
                                       value_type=str)}])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
         parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-d',
            os.path.join(pkg, 'rviz', 'robot_description_publisher.rviz')
        ]
    )
  
    # Spawn
    spawn = Node(package='ros_ign_gazebo', executable='create',
                 arguments=[
                    '-name', 'rm2_sim',
                    '-z', '-0.21',
                    '-y', '0',
                    '-x', '1.000',
                    '-topic', '/robot_description'],
                 output='screen')



    ign_bridge = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
		    os.path.join(pkg, 'launch', 'bridge.launch.py'),),
        )
    
    
    return LaunchDescription([
      DeclareLaunchArgument(
          'ign_args',
            default_value=[os.path.join(pkg, 'worlds', 'cave_world.sdf')]),
        ign_gazebo,
        spawn,
        #ign_bridge,
        state_publisher,
        # rviz,
    ])

