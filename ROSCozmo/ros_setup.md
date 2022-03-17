# ROS Setup for MR Cozmo 



    Unity Version: [e.g. Unity 2020.3.13f1]
    Unity machine OS + version: [Windows 10]
    ROS machine OS + version: [Ubuntu 20.04LTS, ROS Noetic]
    ROS–Unity communication: [Wifi]
    Branch or version: [master]

## ROS Setups

1. Clone this folder to a ROS supported device. 
1.  do ```catkin_make``` and then ```source devel/setup.bash```, then ```rosparam set ROS_IP <YOU_IP>``` to setup global environment variable. Seems that you have to repeat this step for each terminal window, better write a bash script for it.
2. Start roscore: ```roscore```
3. Edit and run the server endpoint, ```rosrun ros_tcp_endpoint default_server_endpoint.py ``` the purpose of this endpoint can be found in the architect diagram on the [ROS_UNITY website](https://github.com/Unity-Technologies/Unity-Robotics-Hub). If connection from Unity is succesful, you will see the following message:

note: in unity, must generate ROS Message in order to subscribe or publish
```
[INFO] [1628543563.635216]: Connection from 192.168.44.72
[INFO] [1628543563.648968]: RegisterSubscriber(color, <class 'unity_robotics_demo_msgs.msg._UnityColor.UnityColor'>) OK
[INFO] [1628543563.679646]: RegisterUnityService(obj_pose_srv, <class 'unity_robotics_demo_msgs.srv._ObjectPoseService.ObjectPoseService'>) OK
[INFO] [1628543563.694487]: RegisterPublisher(pos_rot, <class 'unity_robotics_demo_msgs.msg._PosRot.PosRot'>) OK
```
1. Setup and play in Unity editor. If there is a ros_pos publisher from unity, enter ```rostopic echo pos_rot``` and you can see the live update of the output. 

If the above steps are successful, basic unity-ros communication is setup via local network. For other subscribers and publishers examples, you can check out the official unity tutorials.

### Useful Commands

- change sshd setting in linux: ```sudo vim /etc/ssh/sshd.config```
- resert ssh and sshd service: ```sudo service sshd restart```
- check ssh connection status: ```systemctl status ssh.service```

