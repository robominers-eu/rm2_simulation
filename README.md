# rm2_simulation
Simulation of UPM robot

Launch the simulation:
```
ros2 launch rm2_simulation robominer.launch.py 
```
Send positions to leg 1, angle in radians:
```
ign topic -t "/pos_j_l1" -m ignition.msgs.Double -p "data: ANGLE"
```
