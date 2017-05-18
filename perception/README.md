# Perception Rundown

## Dependencies:
* sudo apt-get install ros-indigo-simple-grasping

## What to run:

1. `roscore`

2. `gazebo`

3. `rosrun rviz rviz` (add a PointCloud2 and a Marker)

4. `roslaunch fetch_api move_group.launch`

5. To create a topic /mock_point_cloud with some .bag file, do 
`rosrun applications pointcloud_demo ~/data/<something.bag>`

6. To verify that /mock_point_cloud is up, in rviz, set PointCloud2 to /mock_point_cloud or use rostopic ping /mock_point_cloud

7. To run our perception demo, do 
`rosrun perception point_cloud_demo cloud_in:=mock_point_cloud`
(replace mock_point_cloud with /head_camera/depth_registered/points if we are using the real robot)

## How to save a new point cloud from the robot:

1. `setrobot astro`

2. (optional). run rviz on the robot and add a PointCloud2 with the head camera as a topic to see the cloud before you save it

3. `rosrun perception save_cloud ~/data/<somenewname.bag>`

## How to change the crops

In rviz, with PointCloud2, you should see a topic /cropped_cloud. This cloud represents the crop that the robot uses to segment the table.

Use `rosparam set crop_{min,max}_{x,y,z}` to change the cropping (i.e. to set the max x to 0.3 do `rosparam set crop_max_x 0.3`)

Also, rosparam get crop_{min,max}_{x,y,z} can get the current parameter set
___
/tray_cloud is the cloud the robot uses to detect the tray handle. The crop volume shows as a blue box from /visualization_marker under the tray namespace. Any points in this box will be a part of /tray_cloud.

Use `rosparam set tray_{min,max}_{x,y,z}` to change this cropping, you should see the blue box change size
