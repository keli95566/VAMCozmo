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

public class CozmoXRFingerToHeadController : MonoBehaviour
{
    [SerializeField]
    ROSConnection ros;

    //int headJointIndex = 7;
    //private ArticulationBody[] articulationChain;
    public ArticulationBody CozmoHead;
    public Transform IndexTipChaser;
    public Transform IndexKnuckleChaser;

    float rotationAngle;
    float rotationChange;

    public float speed = 300f; // Units: degree/s

    private string topicName = "head_rot";
    private float stiffness = 1000;
    private float damping = 1000;
    private float forceLimit = 1000;

    void Start()
    {

        if (CozmoHead != null)
        {
            initializeHeadxDrive();
        }

        rotationChange = 0;
    }



    void FixedUpdate()
    {
        //calculate the euler angle between the index tip chaser and the knuckle chaser against y axis
        // the range of angle is similar to the constraint of the cozmo robot 

        rotationAngle = Mathf.Rad2Deg* Mathf.Atan((IndexTipChaser.position.y-IndexKnuckleChaser.position.y)/(IndexTipChaser.position.z - IndexKnuckleChaser.position.z));
        //XRHeadRot = XRHead.transform.rotation.eulerAngles.x;

       // Debug.Log("Head update: " + rotationAngle);

        int headRotDirection;


        if (rotationAngle == 0)
        {
            headRotDirection = 0;

        }
        else if (rotationAngle > 0)
        {

            headRotDirection = 1;
        }
        else if (rotationAngle < 0 )
        {
            headRotDirection = - 1;
        }
        else
        { 
            headRotDirection = 0;
        }

        rotationChange = headRotDirection * speed * Time.fixedDeltaTime ;


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
            if (rotationAngle >= 0 && rotationAngle < 180)
            {
                currentDrive.target = rotationAngle;

            }
            else
            {
                currentDrive.target = -(360 - rotationAngle);
            }

        }


        //xDrive.target = -XRHeadRot;
        CozmoHead.xDrive = currentDrive;

        publishHeadMoveAngle(currentDrive.target/3);
    }


    public void publishHeadMoveAngle(float angle)
    {
        if (!ros.HasConnectionError)
        {
            HeadRotMsg headRot = new HeadRotMsg(angle);
            ros.Send(topicName, headRot);
        }
    }
}
