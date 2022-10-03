using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using MRCozmo.Math;
using Unity.Robotics;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.CozmoDemo;

public class WayPointsManager : MonoBehaviour
{

    public ROSConnection ros;

    public GameObject WaypointPrefab;
    
    public ArticulationWheelController CozmoWheelControler;
    // control the linear and angular wheel speed in the actual COZMO
    public CozmoWheelControlublisher CozmoWheelPublisher;
    // note,  give the baselink of the robot instead of the parent game object

    private LineRenderer WayPointPathVisualizer;
    private List<Vector3> LinePoints;
    private List<GameObject> ExistingWayPoints;

    string driveStrightTopicName = "drive_straight";
    string turnInPlaceTopicName = "turn_in_place";
    void Start()
    {
        WayPointPathVisualizer = GetComponent<LineRenderer>();
        WayPointPathVisualizer.positionCount = 0;
        LinePoints = new List<Vector3>();
        ExistingWayPoints = new List<GameObject>();

    }

    // Update is called once per frame
    void Update()
    {
        
    }


    #region waypoint path building
    public void AddWayPointFromPointer(MixedRealityPointerEventData eventData)
    {
        if (WaypointPrefab != null)
        {

            var result = eventData.Pointer.Result;
            GameObject waypoint = Instantiate(WaypointPrefab, result.Details.Point, Quaternion.LookRotation(result.Details.Normal));
            Debug.Log("Added Waypoints at .." + result.Details.Point);
            LinePoints.Add(result.Details.Point);
            ExistingWayPoints.Add(waypoint);
        }
    }
    public void AddWayPointFromTouch(HandTrackingInputEventData eventData)
    {
        if (WaypointPrefab != null)
        {

            Vector3 result = eventData.InputData;
            Debug.Log("Adding Waypoints at .." + result);
            GameObject waypoint = Instantiate(WaypointPrefab, result, Quaternion.LookRotation(result));
            LinePoints.Add(result);
            ExistingWayPoints.Add(waypoint);
        }
    }
    public void ResetPath()
    {
        foreach(GameObject wp in ExistingWayPoints)
        {
            Destroy(wp);
        }
        ExistingWayPoints = new List<GameObject>();
        LinePoints = new List<Vector3>();
        WayPointPathVisualizer.positionCount = 0;
    }

    public void ConfirmPath()
    {
        WayPointPathVisualizer.positionCount = LinePoints.Count;
        int ind = 0;
        foreach(Vector3 pt in LinePoints)
        {
            WayPointPathVisualizer.SetPosition(ind, pt);
            ind += 1;   
        }
    }

    public void ExecutePath()
    {
        foreach(Vector3 pt in LinePoints)
        {
            Vector3 robotpos = CozmoWheelControler.BaseJointTransform.position;

            // turn in place and drive towards 
        }
    }
    #endregion

    #region robot simulation
    public void Simulate()
    {
        StartCoroutine(_Simulate());
    }
    public IEnumerator _Simulate()
    {

        // simulate robot going to each points
        
        for(int i =0; i<WayPointPathVisualizer.positionCount;i++)
        {
            Vector3 targetPt = WayPointPathVisualizer.GetPosition(i);
            Vector2 target2D = new Vector2(targetPt.x, targetPt.z);

            Vector2 robotPos2D = new Vector2(CozmoWheelControler.BaseJointTransform.position.x, CozmoWheelControler.BaseJointTransform.position.z);
           // Debug.Log("Current robot position vector: " + robotPos2D);

            Vector2 robotDirection = new Vector2(CozmoWheelControler.BaseJointTransform.forward.x, CozmoWheelControler.BaseJointTransform.forward.z);
          //  Debug.Log("Current robot direction vector: " + robotDirection);

            Vector2 targetDirection = target2D - robotPos2D;
            //Debug.Log("target direction vector: " + targetDirection);


            int turnDirection = RobotMathUtils.GetTurnDirection(robotDirection, targetDirection);
          //  Debug.Log("Turn direction:  " + turnDirection);

            float turnAngle = Mathf.Abs(Vector2.SignedAngle(robotDirection, targetDirection));

            Debug.Log("Turn angle:  " + turnAngle);

            float driveDistance = targetDirection.magnitude;
            Debug.Log("drive distance:  " + driveDistance);

            publishTurnInPlace(turnAngle*turnDirection, 10);
            publishDriveStraight(driveDistance, 10);

            yield return StartCoroutine(TurnAndDrive(turnAngle, turnDirection, driveDistance));
            //yield return new WaitForSeconds(0.1f);
        }


    }

    private IEnumerator TurnAndDrive(float angle, int direction, float distance)
    {
        yield return StartCoroutine(CozmoWheelControler.TurnInPlace(angle, direction));
        yield return StartCoroutine(CozmoWheelControler.DriveStraight(distance, 1));

    }
    #region Math Utils

    public float GetZAxisAngle(Vector3 pt)
    {
        return Mathf.Atan2(pt.x, pt.z)*Mathf.Rad2Deg;
    }
    #endregion

    #endregion

    #region actual execution

    public void publishDriveStraight(float distance_mm, float speed_mmps)
    {

        if (!ros.HasConnectionError)
        {
            DriveStraightMsg msg = new DriveStraightMsg(distance_mm, speed_mmps);
            ros.Send(driveStrightTopicName, msg);
        }

    }

    public void publishTurnInPlace(float angle, float speed)
    {
        if (!ros.HasConnectionError)
        {
            TurnInPlaceMsg msg = new TurnInPlaceMsg(angle, speed);
            ros.Send(turnInPlaceTopicName, msg);
        }
    }

    #endregion

    #region dynamics following

    #endregion
}
