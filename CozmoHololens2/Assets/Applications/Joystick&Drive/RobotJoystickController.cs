using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;

public class RobotJoystickController : MonoBehaviour
{
    public GameObject CozmoRobot;
    public CozmoWheelControlublisher cmdPublisher;

    private float targetLinearSpeed = 0.2f; // m/s
    private float targetAngularSpeed=1; // rad/s

    private ArticulationWheelController wheelController;
    void Start()
    {
        wheelController = CozmoRobot.GetComponent<ArticulationWheelController>();


    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void FixedUpdate()
    {
        var leftHandedControllers = new List<InputDevice>();
        var desiredCharacteristics = InputDeviceCharacteristics.HeldInHand | InputDeviceCharacteristics.Left | InputDeviceCharacteristics.Controller;
        InputDevices.GetDevicesWithCharacteristics(desiredCharacteristics, leftHandedControllers);
        foreach(var device in leftHandedControllers)
        {
            Vector2 axis;
            int turnDirection = 0;
            int driveDirection = 0;
            if (device.TryGetFeatureValue(CommonUsages.primary2DAxis, out axis)){
                Debug.Log("Joy stick Controlling.. ");
                if(axis.x > 0) {
                    turnDirection = 1;
                 }
                else if(axis.x<0)
                {
                    turnDirection = -1;
                }
                else
                {
                    turnDirection = 0;
                }
                if (axis.y > 0)
                {
                    driveDirection = 1;
                }
                else if (axis.y<0)
                {
                    driveDirection = -1;
                }
                else
                {
                    driveDirection = 0;
                }
            }
            wheelController.setRobotVelocity(targetLinearSpeed * driveDirection, targetAngularSpeed * turnDirection);
        }

    }
}
