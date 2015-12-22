README:
joymagnet.launch - runs joy_node, joyvisualization and rvis for position feedback for magnet.  

For directed assembly expt:
A. Ensure camera is centered at the calibration position:
	roslaunch plane_camera_magnet dacalcheck.launch 
B. Run expt with tracking of triangle and pink fluoro + rviz
	(i) Camera: camdirectedass.launch 
	(ii) Kalman Filter: dafilterpose.launch [has disc visualization as well]
	(iii) Joystick to control coils: joycoil.launch (change maxpwm to change current)
	
  Important topics: 
	- rosbag record --lz4 /pg_14434226/image_rect_color 
	- rosbag record --lz4 /pg_14434226/image_rect_color /roboclawcommand /kf_pose/magnetactual /datrackdisc/xyPix

* smaller magnet, may need to increase the strength of the electromagnetic coil - 
