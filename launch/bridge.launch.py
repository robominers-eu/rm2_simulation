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
    namespace = 'rm2_sim'
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = 'rm2_sim'
    ign_model_prefix = '/model/' + robot_name

    # clock bridge
    clock_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                        namespace=namespace,
                        name='clock_bridge',
                        output='screen',
                        arguments=['/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock'],
                        )

    # cmd_vel bridge
    cmd_vel_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                          namespace=namespace,
                          name='cmd_vel_bridge',
                          output='screen',
                          parameters=[{
                              'use_sim_time': use_sim_time
                          }],
                          arguments=[
                              '/cmd_vel' + '@geometry_msgs/msg/Twist' + ']ignition.msgs.Twist'
                          ],
                          remappings=[
                              ('/cmd_vel', '/cmd_vel')
                          ])

    # odometry bridge
    odometry_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                           namespace=namespace,
                           name='odometry_bridge',
                           output='screen',
                           parameters=[{
                               'use_sim_time': use_sim_time
                           }],
                           arguments=[
                               ign_model_prefix + '/odometry' +
                               '@nav_msgs/msg/Odometry' + '[ignition.msgs.Odometry'
                           ],
                           remappings=[
                               (ign_model_prefix + '/odometry', '/odom')
                           ])

    # joint state bridge
    joint_state_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                              namespace=namespace,
                              name='joint_state_bridge',
                              output='screen',
                              parameters=[{
                                  'use_sim_time': use_sim_time
                              }],
                              arguments=[
                                  '/world/default/model/rm2_sim/joint_state'
                                  + '@sensor_msgs/msg/JointState' + '[ignition.msgs.Model'
                              ],
                              remappings=[
                                  ('/world/default/model/rm2_sim/joint_state', '/joint_states')
                              ])

    # lidar bridge
    lidar_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                        namespace=namespace,
                        name='lidar_bridge',
                        output='screen',
                        parameters=[{
                            'use_sim_time': use_sim_time
                        }],
                        arguments=[
                            ign_model_prefix + '/laserscan' +
                            '@sensor_msgs/msg/LaserScan' + '[ignition.msgs.LaserScan',
                            ign_model_prefix + '/laserscan/points' +
                            '@sensor_msgs/msg/PointCloud2' + '[ignition.msgs.PointCloudPacked'
                        ],
                        remappings=[
                            (ign_model_prefix + '/laserscan/points', '/scan/points'),
                            (ign_model_prefix + '/laserscan', '/scan')
                        ])

    # color camera bridge
    color_camera_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                               namespace=namespace,
                               name='color_camera_bridge',
                               output='screen',
                               parameters=[{
                                   'use_sim_time': use_sim_time
                               }],
                               arguments=[
                                   ign_model_prefix + '/rgbd_camera' +
                                   '@sensor_msgs/msg/Image' + '[ignition.msgs.Image'
                               ],
                               remappings=[
                                   (ign_model_prefix + '/rgbd_camera', '/rgbd_camera')
                               ])

    pcl2laser_cmd = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laser',
        remappings=[('cloud_in', '/depth_camera/points'),
                    ('scan', '/scan')],
        parameters=[{
            'target_frame': 'rm2_sim/d_435_camera/d435_depth',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -1.5708,  # -M_PI/2
            'angle_max': 1.5708,  # M_PI/2
            'angle_increment': 0.0087,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 4.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
    )

    # depth camera bridge
    depth_camera_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                               namespace=namespace,
                               name='depth_camera_bridge',
                               output='screen',
                               parameters=[{
                                   'use_sim_time': use_sim_time
                               }],
                               arguments=[
                                   ign_model_prefix + '/depth_camera' +
                                   '@sensor_msgs/msg/Image' + '[ignition.msgs.Image',
                                   ign_model_prefix + '/depth_camera/points' +
                                   '@sensor_msgs/msg/PointCloud2' +
                                   '[ignition.msgs.PointCloudPacked',
                                   ign_model_prefix + '/camera_info' +
                                   '@sensor_msgs/msg/CameraInfo' + '[ignition.msgs.CameraInfo',
                               ],
                               remappings=[
                                   (ign_model_prefix + '/depth_camera', '/depth_camera'),
                                   (ign_model_prefix + '/depth_camera/points', '/depth_camera/points'),
                                   (ign_model_prefix + '/camera_info', '/camera_info'),
                               ])

    # odom to base_link transform bridge
    odom_base_tf_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                               namespace=namespace,
                               name='odom_base_tf_bridge',
                               output='screen',
                               parameters=[{
                                   'use_sim_time': use_sim_time
                               }],
                               arguments=[
                                   ign_model_prefix + '/tf' +
                                   '@tf2_msgs/msg/TFMessage' + '[ignition.msgs.Pose_V'
                               ],
                               remappings=[
                                   (ign_model_prefix + '/tf', '/tf')
                               ])

    lidar_stf = Node(package='tf2_ros', executable='static_transform_publisher',
                     namespace=namespace,
                     name='lidar_stf',
                     arguments=[
                         '0', '0', '0', '0', '0', '0', '1',
                         'lidar',
                         'rm2_sim/lidar/front_lidar'
                     ])

    camera_stf = Node(package='tf2_ros', executable='static_transform_publisher',
                      namespace=namespace,
                      name='camera_stf',
                      arguments=[
                          '0', '0', '0', '0', '0', '0', '1',
                              'd_435_camera',
                              'rm2_sim/d_435_camera/d435_depth'
                      ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=['false'],
                              description='use sim time from /clock'),
        clock_bridge,
        cmd_vel_bridge,
        joint_state_bridge,
        odometry_bridge,
        color_camera_bridge,
        depth_camera_bridge,
        odom_base_tf_bridge,
        # camera_stf,
        # pcl2laser_cmd,
        lidar_bridge,
        lidar_stf
    ])
