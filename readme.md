# INS/UWB Integrated Positioning
This ROS package is the implementation of the navigation algorithm illustrated in
the article [**An Approach to Robust INS/UWB IntegratedPositioning for 
Autonomous Indoor Mobile Robots**](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6412300/pdf/sensors-19-00950.pdf), *Liu J, Pu J, Sun L, He Z.*, *Sensors (Basel).
 2019;19(4):950. Published 2019 Feb 23. doi:10.3390/s19040950* using Husky platform
  as benchmark
 
 ##Dependencies 
 In order to use this package you should have [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu) installed in your system.
 Additionally this package depends on some standard python libraries and two other packages used for the UWB Gazebo plugin which are [gazebosensorplugins](https://github.com/valentinbarral/gazebosensorplugins) and [gtech_msgs](https://github.com/valentinbarral/rosmsgs) both by valentinbarral.
 Script dependencies.sh should set everything you need
 ```
 chmod +x ./dependencies.sh && ./dependencies.sh
 ```
##Run the simulation
You should first set all the environmental variables for Gazebo to work correctly by running *setup.bash* in your workspace root folder
```
 chmod +x ./setup.bash && ./setup.bash
``` 
And then launch the script *start_all.sh* which will start:
* Gazebo with Husky model in an empty world with UWB antennas
* The navigation node *node_sage-husa.py*
* rqt_multiplot with *rqt_multiplot.xml* config in order to monitor the filter behaviour
* Publisher for Husky velocity command in order to keep the robot in a circular uniform motion
```
chmod +x ./start_all.sh && ./start_all.sh
```
In order to start visualizing data in rqt plot you should hit the play button on the top right of each graph 
## Folders

###src
Contains Python scripts which performs navigation. In particular *node_sage-husa.py* sets the node up and *shfaf.py* contains the filter class.

###data
Contains data exported as csv of simulation run with different filter configurations

##launch
Contains roslaunch script to spawn husky with IMU and UWB tag (*spawn_husky_uwb.launch, description.launch*),
 Husky controls (*control.launch*), and to launch the empty world with UWB antennas in place (*uwb_empty.launch*)

 ## models
 Contains Husky urdf xacro model
 
 ##worlds
 Contains different world configuration