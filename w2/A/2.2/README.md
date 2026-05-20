## Running the joint-state-publisher

Two terminals with setup.bash sourced.

In 1. run:
```
rviz2
```

In 2. run:
```
ros2 launch fake_encoder fake_encoder.launch.py
```

In rviz change *Global Options -> Fixed Frame* from `map` to base\_link or similar, add a *RobotModel* and change *RobotModel -> Description Topic* to `robot_description`.
