# rm2_simulation
Simulation of UPM robot

Launch the simulation (default mine world):
```
ros2 launch rm2_simulation robominer.launch.py
```

Launch the simulation in empty world (cloning this repository in workspace/src and from workspace/):
```
ros2 launch rm2_simulation robominer.launch.py ign_args:="src/rm2_simulation/worlds/ground_world.sdf"

```

Send positions to leg N (N 1, 2 o 3), angle in radians:
```
ign topic -t "/pos_j_lN" -m ignition.msgs.Double -p "data: ANGLE"
```
E.g.:
```
ign topic -t "/pos_j_l1" -m ignition.msgs.Double -p "data: 0.5"
```

