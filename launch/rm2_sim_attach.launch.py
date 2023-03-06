import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.descriptions import ParameterValue 
from launch_ros.actions import Node
use_sim_time = True

def generate_launch_description():
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg = get_package_share_directory('rm2_simulation')
    path_to_urdf_new = pkg+'/urdf/rm2_module1.urdf.xacro'
    path_to_urdf = pkg+'/urdf/rm2_attachable.urdf.xacro'
    pkg_ign_plugins_att = get_package_share_directory('attachable_ign_plugin')
    pkg_ign_plugins_contact = get_package_share_directory('contact_ign_plugin')
    print(str(os.path.join(pkg_ign_plugins_att, 'AttachableJoint', 'lib') + ', ' 
                                                        + os.path.join(pkg_ign_plugins_contact, 'AttacherContact', 'lib')))
    ign_gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
      launch_arguments={'ign_args': '-r ' + os.path.join(pkg, 'worlds', 'cave_world_attach.sdf')}.items(),)
    


    state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
        'robot_description': ParameterValue(Command(['xacro ', str(path_to_urdf)]),
                                       value_type=str)}],
    namespace="robot",)


  
    # Spawn
    spawn = Node(package='ros_ign_gazebo', executable='create',
                 arguments=[
                    '-name', 'rm2_robot',
                    '-z', '-0.11',
                    '-y', '-0.32',
                    '-x', '2.8',
                    '-topic', '/robot/robot_description'],
                 output='screen')
    

    state_publisher_new = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
        'robot_description': ParameterValue(Command(['xacro ', str(path_to_urdf_new)]),
                                       value_type=str)}],
    namespace="module",)



  
    # Spawn
    spawn_new = Node(package='ros_ign_gazebo', executable='create',
                 arguments=[
                    '-name', 'rm2_module',
                    '-z', '-0.11',
                    '-y', '-0.32',
                    '-x', '3.8',
                    '-topic', '/module/robot_description'],
                 output='screen')
       
    
    ign_bridge = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
		    os.path.join(pkg, 'launch', 'bridge_attach.launch.py'),),
        )
    
    return LaunchDescription([
        SetEnvironmentVariable(name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
                                                 value=[(os.path.join(pkg_ign_plugins_att, 'AttachableJoint', 'lib') + ':' 
                                                        + os.path.join(pkg_ign_plugins_contact, 'AttacherContact', 'lib'))]),
        ign_gazebo,
        ign_bridge,
        state_publisher,
        state_publisher_new,
        spawn_new,
        spawn,
    ])


