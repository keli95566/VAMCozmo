# MRCozmo 

This project explore to operate the [Anki Cozmo](https://www.digitaldreamlabs.com/pages/cozmo) robot in mixed reality (MR) and virtual reality (VR) using OpenXR supported head mounted display (HMD) such as Microsoft Hololens and Oculus Quest 2. 

This document demonstrates some examples and record the future development steps. 

[! Please note that this repository is under continuous development and improvement. Only the functionalities in the ```MainDemoScene``` scene is working. But please feel free to explore other work in progress code base as well!]

## Table of Contents
------------------
[Mixed Reality (Hololens2) Demos](#Hololens2)   

[Virtual Reality (Oculus Quest 2 + Passthrough API) Demos](#Quest2)


[System architecture](#Architecture)


## Mixed Reality Hololens 2 Demos  <a name="Hololens2"></a>
--------------------

### New Hololens2 UI layout:
--------------
<img src="./images/hololens2_UI_layout.PNG"
     alt="hololens2 UI"
     style="float: center; margin-right: 10px; height:300px" />

### Live control of Cozmo Head via Hololens 2 by tracking Index finger tip and index finger knuckle :

--------------
<img src="./images/01_mr_cozomo_hololens2_new_hand_head_control.gif"
     alt="cozmo lift"
     style="float: center; margin-right: 10px; height:300px" />


### Live control of Cozmo Lift via Hololens 2 by tracking Index finger tip and tip of the thumb:

--------------
<img src="./images/01_mr_cozomo_hololens2_new_hand_lift_control.gif"
     alt="cozmo lift"
     style="float: center; margin-right: 10px; height:300px" />


### ROS IP with Mixed Reality Keyboard Input:
--------------
<img src="./images/01_hololens2_ROS_IP_keyboard_input.gif"
     alt="hololens2 UI"
     style="float: center; margin-right: 10px; height:300px" />


## Virtual Reality (Oculus Quest 2 + Passthrough API) Demos  <a name="Quest2"></a>

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

----


Learn more about this project from our contribution at the 5th International Workshop on Virtual, Augmented, and Mixed Reality for HRI ( [VAM-HRI 2022](https://vam-hri.github.io/)): https://openreview.net/pdf?id=HYIes841hJc

If you find this repository useful for your research and work, please cite this work: 

```bibtex
@inproceedings{Li2022TowardsRE,
  title={Towards Robust Exocentric Mobile Robot Tele-Operation in Mixed Reality},
  author={Ke Li and Reinhard Bacher and Wim P. Leemans and Frank Steinicke},
  year={2022}
}
```