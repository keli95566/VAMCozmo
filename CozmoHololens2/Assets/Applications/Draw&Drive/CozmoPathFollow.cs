using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CozmoPathFollow : MonoBehaviour
{
    public LineRenderer PathLineRenderer;
    // control the speed and wheel steering in the MR simulated COZMO
    public ArticulationWheelController CozmoWheelControler;
    // control the linear and angular wheel speed in the actual COZMO
    public CozmoWheelControlublisher CozmoWheelPublisher;
    // note,  give the baselink of the robot instead of the parent game object
    public GameObject CozmoRobot;

    private LineRenderer ActualPathTaken;
    private LineRenderer PathToFollow;

    private bool isSimuating = false;
    private float currentLinearV;
    private float currentAngularV;
    private int lastPoint;

    float maxLinear = 0.220f;   // max linear speed [mm/s]    
    float maxAngular = 9.82f;    // max angular speed [rad/s]
    float waitTime = 0.1f; // wait time for update: s 
    float tolerance = 0.05f; 
    void Start()
    {
        if (CozmoRobot.TryGetComponent<LineRenderer>(out ActualPathTaken))
        {
            ActualPathTaken.positionCount = 0;
            ActualPathTaken.SetWidth(0.005f, 0.005f);
        }
        else
        {
            LineRenderer ActualPathTaken = CozmoRobot.AddComponent<LineRenderer>() as LineRenderer;
            ActualPathTaken.positionCount = 0;
            ActualPathTaken.SetWidth(0.005f, 0.005f);
            //
        }

        PathToFollow = this.GetComponent<LineRenderer>();
        PathToFollow.positionCount = 0;
        PathToFollow.SetWidth(0.005f, 0.005f);
        PathToFollow.SetColors(Color.blue, Color.blue);

    }
    // Update is called once per frame
    void Update()
    {

    }

    private void FixedUpdate()
    {
        if (isSimuating)
        {
            CozmoWheelControler.setRobotVelocity(currentLinearV, currentAngularV);
        }
    }
    #region simulation 
    private void DrawPathToFollow()
    {

        PathToFollow.positionCount = 0;

        Vector3 CozmoStartPos = CozmoRobot.transform.position;
        Vector3 LineStartPos = PathLineRenderer.GetPosition(0);

        PathToFollow.positionCount = PathLineRenderer.positionCount;
        float offsetX = LineStartPos.x*5 - CozmoStartPos.x;
        float offsetZ = LineStartPos.z*5 - CozmoStartPos.z;

        // Zoom in 10 times as the simulated robot is 10 times larger

        for (int i=0; i<PathLineRenderer.positionCount; i++)
        {
            Vector3 point = PathLineRenderer.GetPosition(i);

            PathToFollow.SetPosition(i, new Vector3(point.x*5 - offsetX, CozmoStartPos.y, point.z*5 - offsetZ));
        }
    }
    public void Simulate()
    {
        StartCoroutine(SimulatePathFollowByDrawing());
    }


    private void DrawRobotPath()
    {

        ActualPathTaken.positionCount += 1;
        ActualPathTaken.SetPosition(ActualPathTaken.positionCount - 1, CozmoRobot.transform.position);
    }
    #endregion
    public IEnumerator SimulatePathFollowByDrawing()
    {
        DrawPathToFollow();

        ActualPathTaken.positionCount = 0;
        isSimuating = true;
        currentAngularV = 0;
        currentLinearV = 0;
        if (CozmoWheelControler != null && PathToFollow.positionCount > 2)
        {
            // while robot is not close enough to the last position of the path
            while (Vector3.Distance(CozmoRobot.transform.position, PathToFollow.GetPosition(PathToFollow.positionCount - 1)) > 0.01)
            {
                currentLinearV = maxLinear;
                currentAngularV = 0;
                yield return new WaitForSeconds(waitTime);
                

            }
        }
        isSimuating = false;
    }

    #region Math Utils
    private float GetAngle(Vector2 AB, Vector2 BC)
    {

        return Mathf.Acos((Vector2.Dot(AB, BC) / (AB.magnitude * BC.magnitude)));
    }
    #endregion

    #region prediction utils

    private void findClosestPoint()
    {
       for(int i=lastPoint; i< PathLineRenderer.positionCount-2; i++)
        {
            float dist;

        }
    }

    #endregion

}
