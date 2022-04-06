using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MRCozmo.Math;
using MRCozmo;

/// <summary>
///     This script implements the forward kinematics, inverse kinematics, and various steering behavior for differential drive robots. 
///     angular velocity to joint velocities for
/// </summary>
public class ArticulationWheelController : MonoBehaviour
{

    private float vRight;
    private float vLeft;

    public Transform BaseJointTransform;

    // real spec: 7cm, 1.25cm, 0.9cm
    public float wheelTrackLength;
    public float wheelDistance;
    public float frontWheelRadius;
    public float reerWheelRadius;

    // set mechanics parameters
    public float stiffness = 5;
    public float damping = 10;
    public float forceLimit = 10;


    ArticulationBody leftWheel;
    ArticulationBody rightWheel;
    ArticulationBody leftReerWheel;
    ArticulationBody rightReerWheel;

    int ReerLeftWheelInd = 11;
    int ReerRightWheelInd = 10;
    int FrontRightWheelInd = 12;
    int FrontLeftWheelInd = 13;

    private ArticulationBody[] articulationChain;

    #region simple kinematics
    public bool isTurningInPlace = false; // parameters to track turning in place
    private float fixedTurnSpeed = 5f; //rad/s

    public bool isDrivingStraight = false;     // parameter to track the status of driving straight
    private float fixedDriveSpeed = 0.5f; // m/s    // default linear drive speed
    #endregion

    #region general kinematics
    // parameter to keep a live update of the rotation and position of the robots
    // (x,y,alpha) which describes the robot states
    private float startRot;
    private float currentRot;
    private Vector2 startPos;
    private Vector2 currentPos;

    //  
    public bool isDriving;
    public bool isSeeking;
    public float currentLinearSpeed=0.2f;
    public float currentAngularSpeed=5f;
    private float maxLinearSpeed = 0.3f; // m/s=> takes care though, in the real cozmo robot, the maximum linear speed is only 0.22 m/s
    private float maxAngularSpeed = 9;// rad/s
    #endregion

    #region wheelSpeed
    float frontRightRotation;
    float frontLeftRotation;
    float reerRightRotation;
    float reerLeftRotation;

    #endregion
    void Start()
    {


        articulationChain = this.GetComponentsInChildren<ArticulationBody>();

        leftWheel = articulationChain[FrontLeftWheelInd];
        rightWheel = articulationChain[FrontRightWheelInd];
        leftReerWheel = articulationChain[ReerLeftWheelInd];
        rightReerWheel = articulationChain[ReerRightWheelInd];
        
        InitializeWheelxDrive();

        //ArticulationBody rightBelt = articulationChain[6];
        reerWheelRadius = 0.9f * 3*0.01f;//rightReerWheel.GetComponentInChildren<MeshRenderer>().bounds.size.y / 2;
        frontWheelRadius = 1.314f * 3 * 0.01f;//rightWheel.GetComponentInChildren<MeshRenderer>().bounds.size.y / 2;
        wheelTrackLength = 7 * 3 * 0.01f;//rightBelt.GetComponentInChildren<MeshRenderer>().bounds.size.y;

        wheelDistance = 3*5.5f * 0.01f;//Vector3.Distance(leftWheel.transform.position, rightWheel.transform.position);
    }

    private void FixedUpdate()
    {
        if (isTurningInPlace)
        {
            currentRot = RobotMathUtils.WrapAngle(BaseJointTransform.rotation.eulerAngles.y);
            setRobotVelocity(0,fixedTurnSpeed);
        }

        if (isDrivingStraight)
        {
            currentPos = new Vector2(BaseJointTransform.position.x, BaseJointTransform.position.z);
            setRobotVelocity(fixedDriveSpeed, 0);
        }

        if (isDriving && !isDrivingStraight && !isTurningInPlace)
        {
            currentPos = new Vector2(BaseJointTransform.position.x, BaseJointTransform.position.z);
            setRobotVelocity(currentLinearSpeed, currentAngularSpeed);
        }
    }
    #region simple inverse kinematics methods

    /// <summary>
    /// Correspond to the turn in place method in Cozmo Python SDK, with some small angles of uncertainty +-3 degrees
    /// </summary>
    /// <param name="angle">should be and Euler angle between [0,180] degree</param>
    /// <param name="direction"> 1: turn left, -1 turn towards right</param>
    public IEnumerator TurnInPlace(float angle, int direction)
    {
        fixedTurnSpeed = Math.Abs(fixedTurnSpeed) * direction;
        isTurningInPlace = true;
        startRot = RobotMathUtils.WrapAngle(BaseJointTransform.rotation.eulerAngles.y);
        yield return new WaitUntil(() => Mathf.Abs(currentRot - startRot) >= angle - 0.8);
        isTurningInPlace = false;
        setRobotVelocity(0, 0);
        currentRot = RobotMathUtils.WrapAngle(BaseJointTransform.rotation.eulerAngles.y);
        currentPos = new Vector2(BaseJointTransform.position.x, BaseJointTransform.position.z);
        //turnSpeed = 5f; // set back to default;
    }

    /// <summary>
    /// Correspond to the turn in place method in Cozmo Python SDK, with some small angles of uncertainty +-3 degrees
    /// </summary>
    /// <param name="distance"> distance in meters</param>
    /// <param name="direction"> 1: forward, -1 backward</param>
    public IEnumerator DriveStraight(float distance, int direction)
    {
        fixedDriveSpeed = Math.Abs(fixedDriveSpeed) * direction;
        isDrivingStraight = true;
        startPos = new Vector2(BaseJointTransform.position.x, BaseJointTransform.position.z);
        yield return new WaitUntil(() => Vector2.Distance(currentPos, startPos) >= distance);
        isDrivingStraight = false;
        setRobotVelocity(0, 0);
        currentRot = RobotMathUtils.WrapAngle(BaseJointTransform.rotation.eulerAngles.y);
        currentPos = new Vector2(BaseJointTransform.position.x, BaseJointTransform.position.z);
        //linearDriveSpeed = 0.2f; // set back to default
    }

    #endregion

    #region general forward kinematics

    /// <summary>
    /// Predict the future position of the robot based on forward kinematics algorithm. 
    /// source: (https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf)
    /// Assumption that the equation has made: no slip occurs => the surface is flat.
    /// ICC (Instantaneous Center of Curvature)
    /// Angular velocity w: : Each wheel rotates around ICC along a circle with radius r
    /// 
    /// This functionality is underdevelopment is not yet working
    /// </summary>
    /// <param name="waiTime"> time in second ahead in the future</param>
    public Vector3 predictFuturePosition(float waitTime)
    {

        vRight = currentAngularSpeed * (wheelDistance / 2) + currentLinearSpeed;
        vLeft = -currentAngularSpeed * (wheelDistance / 2) + currentLinearSpeed;
        Debug.Log("vright: " + vRight + "vleft: " + vLeft); 
        float futureAngle;
        Vector2 futurePos;
        currentPos = new Vector2(BaseJointTransform.position.x, BaseJointTransform.position.z);
        currentRot = RobotMathUtils.WrapAngle(BaseJointTransform.rotation.eulerAngles.y);
        
        Debug.Log("Beginning Pos: " + currentPos + "  Beginning rot : " + currentRot );

        float R_ICC = 0.5f * wheelDistance*(vRight + vLeft) / (vRight - vLeft); // radius of the ICC
        float w = (vRight - vLeft) / wheelDistance;//  Angular velocity of the robot, rad/s. It is defined as follows: Each wheel rotates around ICC along a circle with radius r.
        Debug.Log("Radius of ICC : " + R_ICC + "angular velocity around icc " + w); 
        float ICC_x = currentPos.x - R_ICC * Mathf.Sin(currentRot*Mathf.Deg2Rad);
        float ICC_y = currentPos.y + R_ICC * Mathf.Cos(currentRot * Mathf.Deg2Rad);
        float x_ = Mathf.Cos(w*waitTime)*(currentPos.x - ICC_x) + Mathf.Sin(w*waitTime)*(currentPos.y-ICC_y) + ICC_x;//future x pos
        float y_ = -Mathf.Sin(w * waitTime) * (currentPos.x - ICC_x) + Mathf.Cos(w * waitTime) * (currentPos.y - ICC_y) + ICC_y;//future x pos
        float futureRot = RobotMathUtils.WrapAngle( (currentRot + w * waitTime)*Mathf.Rad2Deg ); // future rotation of the robot
        return new Vector3(x_, y_, futureRot);
    }
    #endregion

    #region general inverse kinematics

    /// <summary>
    /// Implement Seeking behavior
    /// </summary>
    public void SeekPosition()
    {
        currentPos = new Vector2(BaseJointTransform.position.x, BaseJointTransform.position.z);
        
    }

    #endregion

    #region differential wheel drive

    void InitializeWheelxDrive()
    {
        for (int i = 10; i < 14; i++)
        {
            var xDrive = articulationChain[i].xDrive;
            xDrive.stiffness = stiffness;
            xDrive.damping = damping;
            xDrive.forceLimit = forceLimit;
            xDrive.target = 0;
            articulationChain[i].xDrive = xDrive;
        }

    }


    public void setRobotVelocity(float targetLinearSpeed, float targetAngularSpeed)
    {
        // Stop the wheel if target velocity is 0
        // unit: m/s
        if (targetLinearSpeed == 0 && targetAngularSpeed == 0)
        {
            stopWheel(leftWheel);
            stopWheel(rightWheel);
            stopWheel(leftReerWheel);
            stopWheel(rightReerWheel);
        }
        else
        {
            // Convert from linear x and angular z velocity to wheel speed
            vRight = targetAngularSpeed * (wheelDistance / 2) + targetLinearSpeed;
            vLeft = -targetAngularSpeed * (wheelDistance / 2) + targetLinearSpeed;

            setWheelVelocity(leftWheel, vLeft / frontWheelRadius * Mathf.Rad2Deg);
            setWheelVelocity(rightWheel, vRight / frontWheelRadius * Mathf.Rad2Deg);

            setWheelVelocity(leftReerWheel, vLeft / reerWheelRadius * Mathf.Rad2Deg);
            setWheelVelocity(rightReerWheel, vRight / reerWheelRadius * Mathf.Rad2Deg);

        }

    }

    private void setWheelVelocity(ArticulationBody wheel, float jointVelocity)
    {
        ArticulationDrive drive = wheel.xDrive;
        drive.target = drive.target + jointVelocity * Time.fixedDeltaTime;
       // drive.targetVelocity = jointVelocity;
        wheel.xDrive = drive;
    }

    private void stopWheel(ArticulationBody wheel)
    {
        // Set desired angle as current angle to stop the wheel, this ís like a robot brake
        ArticulationDrive drive = wheel.xDrive;
        drive.target = wheel.jointPosition[0] * Mathf.Rad2Deg;
        wheel.xDrive = drive;
    }
    

    #endregion
}