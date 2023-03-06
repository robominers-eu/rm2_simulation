# Robot description file structure:
* rm2.urdf.xacro: Full robot with lidar and RGBD sensors, no dynamic attach
* rm2_attachable.urdf.xacro: Full robot with lidar and RGBD sensors and dynamic attach
* rm2_module.urdf.xacro: Full robot with dynamic attach, no lidar and RGBD sensors
* name.xacro (without urdf exension) and rm2.gazebo are macro files

# Usage:
* Robot navigation: rm2.urdf.xacro
* Robot self-assembly: m2_attachable.urdf.xacro and rm2_module.urdf.xacro
