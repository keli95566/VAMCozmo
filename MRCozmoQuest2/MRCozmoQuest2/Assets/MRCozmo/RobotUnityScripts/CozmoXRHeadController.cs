using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.CozmoDemo;
using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using Microsoft.MixedReality.Toolkit.UI;

public class CozmoXRHeadController : MonoBehaviour
{
    [SerializeField]
    ROSConnection ros;

    //int headJointIndex = 7;
    //private ArticulationBody[] articulationChain;
    public ArticulationBody CozmoHead;

    float XRHeadRot;
    float rotationChange;

    public float speed = 300f; // Units: degree/s
    public Transform XRHead;

    private string topicName = "head_rot";
    private float stiffness = 1000;
    private float damping = 1000;
    private float forceLimit = 1000;

    void Start()
    {
        TryGetMRHead();

        initializeHeadxDrive();
    }

    void TryGetMRHead()
    {
        OVRCameraRig rig = MixedRealityInputSystemProfile.FindObjectOfType<OVRCameraRig>();

        XRHead = rig.transform.Find("TrackingSpace/CenterEyeAnchor");
    }


    void FixedUpdate()
    {
        XRHeadRot = XRHead.transform.rotation.eulerAngles.x;

        Debug.Log("Head update: "+ XRHeadRot) ;
        int headRotDirection;
        if (XRHeadRot == 0)
        {
            headRotDirection = 0;

        }else if (XRHeadRot > 180)
        {

            headRotDirection = -1;
        }
        else if (XRHeadRot >0 && XRHeadRot<180)
        {
            headRotDirection = 1;
        }
        else
        {
            headRotDirection = 0;
        }

        rotationChange = headRotDirection * speed * Time.fixedDeltaTime;


        MoveHeadUpandDown();

    }
    void initializeHeadxDrive()
    {
        var xDrive = CozmoHead.xDrive;
        xDrive.stiffness = stiffness;
        xDrive.damping = damping;
        xDrive.forceLimit = forceLimit;
        CozmoHead.xDrive = xDrive;
    }
    void MoveHeadUpandDown()
    {

        var currentDrive = CozmoHead.xDrive;


        if (rotationChange + currentDrive.target > currentDrive.upperLimit)
        {
            currentDrive.target = currentDrive.upperLimit;

        }
        else if (rotationChange + currentDrive.target < currentDrive.lowerLimit)
        {
            currentDrive.target = currentDrive.lowerLimit;

        }
        else
        {
            if (XRHeadRot >= 0 && XRHeadRot < 180)
            {
                currentDrive.target = XRHeadRot;

            }
            else
            {
                currentDrive.target = -(360 - XRHeadRot);

            }

        }

        
        //xDrive.target = -XRHeadRot;
        CozmoHead.xDrive = currentDrive;

        if (!ros.HasConnectionError)
        {
            HeadRotMsg headRot = new HeadRotMsg(currentDrive.target);
            ros.Send(topicName, headRot);
        }
    }
}
