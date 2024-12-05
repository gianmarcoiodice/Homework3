## FIRST POINT
In this point we created another gazebo world named empty_new.world.\n
1.a -> gazebo world
```
ros2 launch iiwa_bringup iiwa.launch_new.py command_interface:="velocity" robot_controller:="velocity_controller" use_sim:=true use_vision:=true
```
1.b -> spherical object view
```
ros2 launch iiwa_bringup iiwa.launch_new.py command_interface:="velocity" robot_controller:="velocity_controller" use_sim:=true use_vision:=true
```
```
$ ros2 run rqt_image_view  rqt_image_view 
```
1.c -> blob detection
```
ros2 launch iiwa_bringup iiwa.launch_new.py command_interface:="velocity" robot_controller:="velocity_controller" use_sim:=true use_vision:=true
```
```
$ ros2 run rqt_image_view  rqt_image_view 
```
```
ros2 run ros2_opencv ros2_open_cv_node 
```
## SECOND POINT
2.a
i) here we spawn the gazebo world with the ArUco tag inside and we exectue the positioning task. Youtube link :https://youtu.be/dgOmxvEkXrY
```
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_sim:=true use_vision:=true
```
```
ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=201
```
```
ros2 run rqt_image_view rqt_image_view 
```
```
ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:=velocity --ros-args -p task:=positioning
```
ii) stop the last terminal and then run: look-at-point with velocity command. Youtube link: https://www.youtube.com/watch?v=EEsBwHIai5A 
```
ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p task:=look-at-point
```

2.b 
a) look at point with effort - Joint space control. Youtube link : https://www.youtube.com/watch?v=TxIn_ssuXvQ
```
$ ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller" use_sim:=true use_vision:=true
```
```
$ ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=201
```
```
$ ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:=effort -p task:=look-at-point
```
it is also possible to run the simulation with the computed torques in the cartesian space with the same command, 
by substituting the effort value sent to the controller (from code) Youtube link: \n

b) task:=linear position trajectory and the look-at-point vision-based , torques calculated due to the joint space. Youtube link : https://www.youtube.com/watch?v=b3cn5bcMcGs
```
$ ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller" use_sim:=true use_vision:=true
```
```
$ ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=201
```
```
$ ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:=effort -p task:=trajectory_lap
```


