gnome-terminal -- roslaunch ins_uwb_positioning uwb_empty.launch
sleep 5
gnome-terminal -- rosrun ins_uwb_positioning node_sage-husa.py
gnome-terminal -- rosrun rqt_multiplot rqt_multiplot --multiplot-config file://$(rospack find ins_uwb_positioning)/rqt_multiplot.xml
gnome-terminal -- rostopic pub /husky_velocity_controller/cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2" -r10


