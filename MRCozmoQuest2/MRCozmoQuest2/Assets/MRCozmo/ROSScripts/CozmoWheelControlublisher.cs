using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.MRCozmo;
using System.Linq;

public class CozmoWheelControlublisher : MonoBehaviour
{
    ROSConnection ros;
    public string twistTopicName = "base_controller/cmd_vel";

    public string driveStrightTopicName = "drive_straight";
    public string turnInPlaceTopicName = "turn_in_place";

    public string drivePathTopicName = "drive_path";

    // Start is called before the first frame update
    void Start()
    {
        ros = GetComponentInParent<ROSConnection>();
    }

    public void publishVelCmd(float targetLinearSpeed, float targetAngularSpeed)
    {
        TwistMsg twistMsg = new TwistMsg();
        Vector3Msg angular = new Vector3Msg();
        angular.x = 0;
        angular.y = 0;
        angular.z = targetAngularSpeed;

        Vector3Msg linear = new Vector3Msg();
        linear.x = targetLinearSpeed;
        linear.y = 0;
        linear.z = 0;
        twistMsg.angular = angular;
        twistMsg.linear = linear;
        ros.Send(twistTopicName, twistMsg);
    }

    public void publishDriveStraight(float distance_mm, float speed_mmps)
    {
        DriveStraightMsg msg = new DriveStraightMsg(distance_mm, speed_mmps);

        ros.Send(driveStrightTopicName, msg);
    }

    public void publishTurnInPlace(float angle, float speed)
    {
        TurnInPlaceMsg msg = new TurnInPlaceMsg(angle, speed);

        ros.Send(turnInPlaceTopicName, msg);
    }

    public void PublishPath(DrivePathMsg path)
    {

        ros.Send(drivePathTopicName, path);
    }
}
