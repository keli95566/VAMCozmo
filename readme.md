# MRCozmo 

This project explore to operate the [Anki Cozmo](https://www.digitaldreamlabs.com/pages/cozmo) robot in mixed reality (MR) and virtual reality (VR) using OpenXR supported head mounted display (HMD) such as Microsoft Hololens and Oculus Quest 2. 

This document demonstrates some examples and record the future development steps. The code will be released soon..

## Table of Contents
------------------
[Application Demos](#Demos)   
 
[System architecture](#Architecture)


## Demos  <a name="Demos"></a>
--------------------


### Raycast way point selection and path following  
-------------
<img src="./images/01_waypointPathfollow.gif"
     alt="cozmo lift"
     style="float: center; margin-right: 10px; height:320px" />


### MR Robot Path Planning using Oculus Quest 2 Passthrough API 
------

<img src="./images/passthrough-cut.gif"
     alt="cozmo lift"
     style="float: center; margin-right: 10px; height:290px" />


### Free hand path planning and drawing in VR:
--------------
<img src="./images/freehand_drawing.gif"
     alt="cozmo lift"
     style="float: center; margin-right: 10px; height:280px" />


### Free Hand Lift Control Using [Hololens One Hand Ruler](https://github.com/HiromuKato/MRTK_HKSample)  <a name="freehandLift"></a>


### In simulation in Unity editor or in VR:
--------------
<img src="./images/01_cozmoLiftOneHandControl.gif"
     alt="cozmo lift"
     style="float: center; margin-right: 10px; height:250px" />


### Demo of live control of Cozmo via Hololens 2:
--------------
<img src="./images/01_cozmoLiftHololensHandConrtollerTest.gif"
     alt="cozmo lift"
     style="float: center; margin-right: 10px; height:300px" />


### Lift Control with VR controller 
----------------
<img src="./images/log4_lift_sync.gif"
     alt="cozmo lift"
     style="float: center; margin-right: 10px; height:260px" />

### Turning head angle with VR headset
--------------
<img src="./images/log4_head_sync.gif"
     alt="cozmo lift"
     style="float: center; margin-right: 10px; height:260px" />

### Live 2D video streaming  
----------------
<img src="./images/RotatingCozmo.gif"
     alt="cozmo lift"
     style="float: center; margin-right: 10px; height:260px" />




## System Architecture <a name="Architecture"></a>
-------------------
The robot model was imported to Unity using [URDF Importer](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/urdf_importer/urdf_tutorial.md)

<img src="./images/correct_urdf.gif"
     alt="cozmo lift"
     style="float: center; margin-right: 10px; height:200px" />


The communcication between ROS and HMD is built on top of the [Unity ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) 


<img src="./images/systemOverview.PNG"
     alt="cozmo lift"
     style="float: center; margin-right: 10px; height:300px" />

