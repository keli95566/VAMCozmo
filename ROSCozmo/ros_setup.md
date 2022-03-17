# ROS Setup for MR Cozmo 



    Unity Version: [e.g. Unity 2020.3.13f1]
    Unity machine OS + version: [Windows 10]
    ROS machine OS + version: [Ubuntu 20.04LTS, ROS Noetic]
    ROS–Unity communication: [Wifi]
    Branch or version: [master]

## Prerequisites:

```
pip install cozmo
```

## ROS Setups

1. Install ROS Noetic
1. Clone this folder to a ROS supported device. 
2.  Do ```catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.7m``` , 
3. Then ```source devel/setup.bash```.
3.  Set the ROS IP  via ```rosparam set ROS_IP <YOU_IP>``` to setup global environment variable. [repeat this step for each terminal window]. (Note, in Linux, you could find your ip address via ```ip addr```)
4. Start roscore: ```roscore```

5. Open a new terminal,  repeat 2 and 3 for the new terminal, and then run the server endpoint, ```rosrun ros_tcp_endpoint default_server_endpoint.py ``` the purpose of this endpoint can be found in the architect diagram on the [ROS Unity Integration Documentation Webpage](https://github.com/Unity-Technologies/Unity-Robotics-Hub).  
If succeeds, the terminal outputs the following:

    ```
    [INFO] [1647511270.825488]: Starting server on 192.168.178.33:10000
    ``` 

6. Connect the mobile phone that communicates with the Cozmo robot  via a USB cable. Enable the SDK mode of the Cozmo robot. 

7. Then open a new terminal, repeat 2&3 for this new terminal, and then run   ```rosrun cozmo_driver cozmo_driver.py  ``` If succedds, the terminal outputs the following:

    ```
    /home/keli/.local/lib/python3.8/site-packages/cozmo/event.py:488: DeprecationWarning: The loop argument is deprecated since Python 3.8, and scheduled for removal in Python 3.10.
    return await asyncio.wait_for(f, timeout, loop=self._loop)
    2022-03-17 11:08:14,560 cozmo.general INFO     App connection established. sdk_version=1.4.10 cozmoclad_version=3.4.0 app_build_version=00003.00004.00000
    2022-03-17 11:08:14,560 cozmo.general INFO     Found robot id=1
    2022-03-17 11:08:14,567 cozmo.general INFO     Connected to iOS device_id=1 serial=....................
    2022-03-17 11:08:14,692 cozmo.general INFO     Robot id=1 serial=432050fd initialized OK
    [INFO] [1647511695.852934]: camera calibration URL: file:///home/keli/.ros/camera_info/cozmo_camera.yaml
    
    ```

If all the setup works, now the COZMO is ready to communicate with any Unity application in the local network, such as Unity Editor, or a standalone VR/MR head mounted displays. 
    