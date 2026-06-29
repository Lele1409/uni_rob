## dependency

Here the dependencies that of course aren't mentioned in task given to us. :)

```
sudo apt install ros-jazzy-nav2-map-server
```

## rviz setup

add map from topic `/map`. 

set Fixed Frame to map.

add LaserScan from topic `/simulated_scan`

click on 2D Goal Pose and hold on any part of the map to choose a direction.

The points from the scan might be a bit hard to see. Couldn't find an easy way to fix that. (No colour setting?)
