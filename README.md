# CSE 481C
Starter code and samples for CSE 481C at the University of Washington, Spring 2017.

Labs and other documentation are on the **[wiki](https://github.com/cse481sp17/cse481c/wiki)**.

## Launching web interface:

1. `roscore`
2. `gazebo`
3. `frontend`
4. `backend`
5. Visit http://localhost:8080/ and use the Websocket URL: ws://localhost:9090

## Launching map annotator RViz web interface:

1. `roscore`
2. `gazebo`
3. `rviz`
4. `backendrviz`
5. RViz: in the InteractiveMarkers -> Update Topic dropdown, select '/simple_server/update'
6. `frontendrviz`
7. Visit http://localhost:8080/ and use the Websocket URL: ws://localhost:9090

## Dependencies 

sudo apt-get install ros-indigo-simple-grasping
