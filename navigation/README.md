Navigation Docs
===============

This README guides you through the map building process in the real world using Astro.

In a tmux terminal, run the following commands...


---
To build map in the real world, run the following commands (in tmux if you want to keep your sanity)...

STEP 1: Breathe.  <br />
STEP 2: 'setrobot astro'  <br />
[ONLY DO IF NEED TO CONFIG RVIZ] STEP 3: `roslaunch fetch_navigation build_map.launch`  <br />
[ONLY DO IF NEED TO CONFIG RVIZ] STEP 4: `rosrun rviz rviz`  <br />
[ONLY DO IF NEED TO CONFIG RVIZ] STEP 5: Make sure the Rviz configuration has all the views specified in Lab 16.  <br />
* Robot model
* Grid
* Map
* Laser scan
* Image
* Fixed frame = map  <br />

[ONLY DO IF NEED TO CONFIG RVIZ] STEP 6: shutdown rviz and build_map.launch  <br />
STEP 7: `roslaunch applications build_map_real.launch`  <br />
STEP 8: NOW, drive the robot around to bulid a map of the world! How excite.  <br />
STEP 9: Happy with your map? Now run `rosrun map_server map_saver -f ~/maps/your_name_of_map_file_here`  <br />
STEP 10: DONEZO  <br />



---

To build map in simulation, run the following commands (in tmux if you want to keep your sanity)...

STEP 1: `roscore` <br />
STEP 2: `gazebo` <br />
[ONLY DO IF NEED TO CONFIG RVIZ] STEP 3: `roslaunch fetch_navigation build_map.launch`  <br />
[ONLY DO IF NEED TO CONFIG RVIZ] STEP 4: `rosrun rviz rviz`  <br />
[ONLY DO IF NEED TO CONFIG RVIZ] STEP 5: Make sure the Rviz configuration has all the views specified in Lab 16.  <br />
* Robot model
* Grid
* Map
* Laser scan
* Image
* Fixed frame = map  <br />

[ONLY DO IF NEED TO CONFIG RVIZ] STEP 6: shutdown rviz and build_map.launch  <br />
STEP 7: `roslaunch applications build_map.launch`  <br />
STEP 8: NOW, drive the robot around to bulid a map of the world! How excite.  <br />
STEP 9: Happy with your map? Now run `rosrun map_server map_saver -f ~/maps/your_name_of_map_file_here`  <br />
STEP 10: DONEZO  <br />


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

