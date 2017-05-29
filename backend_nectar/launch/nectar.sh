rosrun web_nectar server.py
pushd ~/catkin_ws/src/team1/web_nectar; python -m SimpleHTTPServer 3000 &
roslaunch backend_nectar nectar.launch



