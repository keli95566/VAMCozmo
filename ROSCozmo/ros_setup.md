# ROS Setup for MR Cozmo 



    Unity Version: [e.g. Unity 2020.3.13f1]
    Unity machine OS + version: [Windows 10]
    ROS machine OS + version: [Ubuntu 20.04LTS, ROS Noetic]
    ROS–Unity communication: [Wifi]
    Branch or version: [master]

## ROS Setups

1. Clone this folder to a ROS supported device. 
2.  Do ```catkin_make``` , then ```source devel/setup.bash```.
3.  Set the ROS IP  via ```rosparam set ROS_IP <YOU_IP>``` to setup global environment variable. [repeat this step for each terminal window]. (Note, in Linux, you could find your ip address via ```ip addr```)
4. Start roscore: ```roscore```
5. Open a new terminal, set the ROS IP via ```rosparam set ROS_IP <YOU_IP>```, and then run the server endpoint, ```rosrun ros_tcp_endpoint default_server_endpoint.py ``` the purpose of this endpoint can be found in the architect diagram on the [ROS Unity Integration Documentation Webpage](https://github.com/Unity-Technologies/Unity-Robotics-Hub). 
6. Connect the Cozmo to the ROS PC via USB cable. 