Navigation Docs
===============

This README guides you through the map building process in the real world using Astro.

In a tmux terminal, run the following commands...


---

*Make sure everything is set to astro before running!*

```
setrobot astro
```
```
roslaunch fetch_navigation build_map.launch
```

---

Now, teleoperate the robot around.
```
rosrun rviz rviz
```
From here, open the `map_creator.rviz` config file in RViz.

---

* Once happy with map created by driving the robot, save the map:
```
rosrun map_server map_saver -f maps/map_name
```
