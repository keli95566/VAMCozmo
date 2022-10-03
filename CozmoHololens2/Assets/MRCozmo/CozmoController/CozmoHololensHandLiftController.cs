using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics;
using Unity.Robotics.ROSTCPConnector;
public class CozmoHololensHandLiftController : MonoBehaviour
{
    [SerializeField]
    ROSConnection ros;
    public CozmoLiftRotPublisher liftRotPublisher;

    public ArticulationBody CozmoLift;
    float targetLiftAngle;
    float rotationChange;

    // if publish to ros. Only publish if ROS Connection is established
    public bool publish = true;

    public float stiffness = 0;
    public float damping = 0;
    public float forceLimit = 100;
    public float speed = 10f; // Units: degree/s

    public OneHandRuler ruler;

    // cozmo physical parameters
    float armLength = 6.6f; //cm
    float liftPivotHeight = 4.5f;
    float maxLiftHeight = 9.2f;
    float minLiftHeight = 3.2f;
    void Start()
    {
        if (CozmoLift != null && CozmoLift.isActiveAndEnabled)
        {
            initializeLiftxDrive();
        }
        Debug.Log("Initialize Hnad Lift Control...");
    }

    private float height2rot(float height)
    {
        
        float minAngle = Mathf.Rad2Deg * Mathf.Asin((minLiftHeight- liftPivotHeight)/armLength);
        float maxAngle = Mathf.Rad2Deg * Mathf.Asin((maxLiftHeight - liftPivotHeight) / armLength);

        float liftAngle = Mathf.Asin(height / armLength) * Mathf.Rad2Deg;
       
        if (float.IsNaN(liftAngle))
        {
            return maxAngle;
        }
        else if(liftAngle > maxAngle)
        {
            return maxAngle;
         }
        else if (liftAngle < minAngle)
        {
            return minAngle;
        }
        else
        {
            return liftAngle;
        }

    }


    private float normalizePublishValue(float height)
    {
       
        return (height - minLiftHeight) / (maxLiftHeight - minLiftHeight);
    }
    void FixedUpdate()
    {


        //measure the supposed moving distance of hand joints

        float measurement = ruler.rightDistance;
        targetLiftAngle = height2rot(measurement);

        if (!ros.HasConnectionError)
        {

            liftRotPublisher.publishLiftRot(normalizePublishValue(measurement));
        }

        int liftRotDirection;

        if (targetLiftAngle == 0)
        {
            liftRotDirection = 0;
        }
        else
        {

            liftRotDirection = 1;
        }

        rotationChange = liftRotDirection * speed * Time.fixedDeltaTime;
        if (CozmoLift != null && CozmoLift.isActiveAndEnabled)
        {
            MoveLift();

        }

    }
    void initializeLiftxDrive()
    {
        var xDrive = CozmoLift.xDrive;
        xDrive.stiffness = stiffness;
        xDrive.damping = damping;
        xDrive.forceLimit = forceLimit;
        CozmoLift.xDrive = xDrive;
    }
    void MoveLift()
    {

        var currentDrive = CozmoLift.xDrive;
        currentDrive.target = -targetLiftAngle;
        CozmoLift.xDrive = currentDrive;
    }

}
