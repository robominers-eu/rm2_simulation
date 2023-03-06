# rm2_simulation
Simulation of UPM robot with Gazebo (Ignition) Citadel and ROS2 Foxy.

## Install
Create workspace and clone this repo:

```
source /opt/ros/foxy/setup.bash
mkdir -p ~/rm2_ws/src
cd ~/rm2_ws/src
git clone git@github.com:robominers-eu/rm2_simulation.git
cd ..
rosdep update
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy 
colcon build --symlink-install
source install/setup.bash
```

## Launch the simulation
* First execute the simulation
```
 ros2 launch rm2_simulation rm2_sim.launch.py  
```

* Teleoperate the robot:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
## To generate an updated URDF

Make sure you have installed xacro, install it using:
```
install sudo apt install ros-foxy-xacro 
```

Navigate to the appropiate folder:
```
cd ~/rm2_ws/src/rm2_simulation/urdf
```
Execute the conversion:
```
xacro rm2.urdf.xacro > rm2.urdf
```
The generated URDF is available in that folder.

## Attachable links
First make sure you have the appropiate repositories:
```
cd ~/rm2_ws/src/
vcs import < rm2_simulation/resources.repos
```

Then launch the simulation:
```
ros2 launch rm2_simulation rm2_sim_attach.launch.py 
```

Create the attachable link between two bodies:
```
ros2 topic pub --once /AttachableJoint std_msgs/msg/String "data: '[rm2_robot][base_link][rm2_module][m1_base_link][attach]'"
```
To detach the link:
```
ros2 topic pub --once /AttachableJoint std_msgs/msg/String "data: '[rm2_robot][base_link][rm2_module][m1_base_link][detach]'"
```
