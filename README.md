# rm2_simulation
Simulation of UPM robot with Gazebo (Ignition) Citadel and ROS2 Foxy.

## Install
Create workspace and clone this repo:

```
mkdir -p ~/plan_ws/src
cd ~/rm2_ws/src
git clone git@github.com:robominers-eu/rm2_simulation.git
cd ..
rosdep update
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy 
colcon build --symlink-install
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

* If you want to use it with Nav2, follow the installation instructions from [nav_mine](https://github.com/robominers-eu/nav_mine) repo and launch it.
```
ros2 launch nav_mine navigation2.launch.py
```

* In rviz2, use `2D Pose Estimate` to provide initial pose of the bot to amcl so that it can start publishing the robot's pose & `map->odom tf`.
 
* After providing the initial pose of the bot you will see the rviz2 window updating with estimated robot pose from amcl as well as updated global and local costmap. Use the `2D Goal Pose` in rviz2 to provide the bot with a goal pose to start navigation.

