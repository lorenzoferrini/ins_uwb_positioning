pip install numpy
pip install numpy-quaternion
pip install scikit-fuzzy
cd ..
sudo apt-get install ros-melodic-rqt-multiplot
rospack find gazebosensorplugins && echo sensor plugin already downloaded || git clone https://github.com/valentinbarral/gazebosensorplugins.git
rospack find gtec_msgs && echo gtec messages already downloaded || git clone https://github.com/valentinbarral/rosmsgs.git
catkin build