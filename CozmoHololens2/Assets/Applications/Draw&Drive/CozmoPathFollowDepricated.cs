using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using RosMessageTypes.CozmoDemo;

namespace MRCozmo
{
    public class CozmoPathFollow : MonoBehaviour
    {
        public LineRenderer PathLineRenderer;
        // control the speed and wheel steering in the MR simulated COZMO
        public ArticulationWheelController CozmoWheelControler;
        // control the linear and angular wheel speed in the actual COZMO
        public CozmoWheelControlublisher CozmoWheelPublisher;
        // note,  give the baselink of the robot instead of the parent game object
        public GameObject CozmoRobot;


        private LineRenderer SimulatedLine;
        private float updateTime = 0.1f; // s;

        // current cozmo position

        private Transform prevCozmoTransform;
        // num of point counts
        private bool isSimuating = false;
        private float currentLinearV;
        private float currentAngularV;


        // cozmo robot configurations

        float maxLinear = 0.220f;   // max linear speed [mm/s]    
        float maxAngular = 9.82f;    // max angular speed [rad/s]

        void Start()
        {
            prevCozmoTransform = CozmoRobot.transform;
            // add line renderer
            if (CozmoRobot.TryGetComponent<LineRenderer>(out SimulatedLine))
            {
                SimulatedLine.positionCount = 0;
                SimulatedLine.SetWidth(0.005f, 0.005f);
            }
            else
            {
                LineRenderer SimulatedLine = CozmoRobot.AddComponent<LineRenderer>() as LineRenderer;
                SimulatedLine.positionCount = 0;
                SimulatedLine.SetWidth(0.005f, 0.005f);
                // SimulatedLine.SetColors(Color.red, Color.blue);
            }

        }
        void FixedUpdate()
        {
            if (isSimuating)
            {
                CozmoWheelControler.setRobotVelocity(currentLinearV, currentAngularV);
            }
        }

         #region simulation

        public void Simulate()
        {
            StartCoroutine(SimulatePathFollow());
        }

        private float GetAngle(Vector2 AB, Vector2 BC)
        {
            
            return Mathf.Acos((Vector2.Dot(AB, BC)/(AB.magnitude*BC.magnitude)));
        }

        private void DrawRobotPath()
        {

            SimulatedLine.positionCount += 1;
            SimulatedLine.SetPosition(SimulatedLine.positionCount - 1, CozmoRobot.transform.position);
        }

        public IEnumerator SimulatePathFollow()
        {
            SimulatedLine.positionCount = 0;
            isSimuating = true;
            CozmoRobot.transform.position = new Vector3(0, 0, 0);
            if(CozmoWheelControler!=null && PathLineRenderer.positionCount > 2)
            {
                int numPoints = PathLineRenderer.positionCount;

                Debug.Log("[Draw&Drive]: Number of draw points:  " + numPoints);
                // The initial goal is to draw similar shape defined by the path. T
                // For the future, one could still track the exact robot location and state to have actual path following in space. 
                // but for simplicity, we skip the tracking as it requires set up markers, and take the first point in the path as the initial robot state


                for (int i = 1; i < numPoints-1; i+=1)
                {
                    // assume that drawing on a plane, ignore the y components

                    Vector3 OA = PathLineRenderer.GetPosition(i-1);
                    Vector3 OB = PathLineRenderer.GetPosition(i);
 
                    Vector3 AB = OB-OA;
                    
                    Vector2 OA2 = new Vector2(OA.x, OA.z);
                    Vector2 OB2 = new Vector2(OB.x, OB.z);

                    float alpha = GetAngle(OA2, OB2);
                    // use max rad/s speed to calculate wait time
                    float waitTimeAngular = alpha / maxAngular;

                    // find angle to turn: Zoom 5 times
                    float dist = AB.magnitude;

                    float waitTimeLinear = (dist) / maxLinear;


                    if (waitTimeAngular <10 && waitTimeAngular <10)
                    {
                        Debug.Log("wait time linear: " + waitTimeLinear + "wait time angular: " + waitTimeAngular);
                        // set linear and angular speed

                        currentLinearV = 0;
                        currentAngularV = maxAngular*Mathf.Rad2Deg/8;

                        // turn and wait until finish execution for angular changes

                        CozmoWheelPublisher.publishVelCmd(0, maxAngular/15);
                        yield return new WaitForSeconds(waitTimeAngular*15);
                        CozmoWheelPublisher.publishVelCmd(0, 0);


                        currentAngularV = 0;
                        currentLinearV = maxLinear;

                        CozmoWheelPublisher.publishVelCmd(maxLinear*1000, 0);
                        yield return new WaitForSeconds(waitTimeLinear);

                        CozmoWheelPublisher.publishVelCmd(0, 0);

                        // draw the traveled path in simuation
                        DrawRobotPath();

                    }

                }
            }
            isSimuating = false;
            currentAngularV = 0;
            currentLinearV = 0;
            CozmoWheelPublisher.publishVelCmd(0, 0);

        }

        #endregion

        #region pathFollowingPublish

        public void ExecutePath()
        {
            int arrLength = PathLineRenderer.positionCount;

            PointMsg [] points = new PointMsg [1000];

            int num_points = 0;

            // only inlcude the way points if distance is larger than 2 cm 
            int Apos=0;
            int Bpos = 0;

            for (int i = 1; i < arrLength-1; i++)
            {
                Vector3 OA = PathLineRenderer.GetPosition(Apos);
                Vector3 OB = PathLineRenderer.GetPosition(Bpos);

                Vector3 AB = OB - OA;

                Vector2 OA2 = new Vector2(OA.x, OA.z);
                Vector2 OB2 = new Vector2(OB.x, OB.z);

                float alpha = GetAngle(OA2, OB2);

                float dist = AB.magnitude;

                if(dist>0.02 || alpha > 0.50)
                {

                    PointMsg pt = new PointMsg(alpha, dist * 1000, 0);
                    points[num_points] = pt;
                    Apos = i;
                    Bpos = i + 1;
                    num_points += 1;
                }
                else
                {
                    Bpos += 1;
                }

            }

            DrivePathMsg drivePath = new DrivePathMsg(points, num_points);
            CozmoWheelPublisher.PublishPath(drivePath);
            
        }
        #endregion  
    }
}