gnome-terminal -- roslaunch ins_uwb_positioning uwb_empty.launch
sleep 5
gnome-terminal -- rosrun rosrun ins_uwb_positioning node_sage-husa.py
gnome-terminal -- rqt 
