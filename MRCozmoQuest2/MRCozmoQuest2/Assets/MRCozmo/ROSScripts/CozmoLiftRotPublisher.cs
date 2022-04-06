using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.MRCozmo;

public class CozmoLiftRotPublisher : MonoBehaviour
{
    // publisher param
    private string topicName = "lift_rot";
    ROSConnection ros;
    void Start()
    {
        ros = GetComponentInParent<ROSConnection>();
        //ros = ROSConnection.instance;
    }

    public void publishLiftRot(float XRLiftRot)
    {

        LiftRotMsg liftRot = new LiftRotMsg(XRLiftRot);
        ros.Send(topicName, liftRot);

    }
}
