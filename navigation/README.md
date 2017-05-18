#Navigation Docs

To build a map:
* In one terminal:
* setrobot astro
* roslaunch fetch_navigation build_map.launch*
* #not teleoperate the robot around*
* #once happy with map save the map:*
* rosrun map_server map_saver -f maps/map_name*
