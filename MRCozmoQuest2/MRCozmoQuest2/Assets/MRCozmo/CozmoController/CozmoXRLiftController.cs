using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.MRCozmo;
public class CozmoXRLiftController : MonoBehaviour
{
    public CozmoLiftRotPublisher liftRotPublisher;

    GameObject cozmo;

    int liftJointIndex = 4;
    private ArticulationBody[] articulationChain;
    float XRLiftRot;
    float rotationChange;

    public float stiffness = 0;
    public float damping = 0;
    public float forceLimit = 100;
    public float speed = 300f; // Units: degree/s

    // attach the Right or left hand controller here.
    public GameObject XRLift;
    

    void Start()
    {
        articulationChain = cozmo.GetComponentsInChildren<ArticulationBody>();
        initializeLiftxDrive();
    }


    void FixedUpdate()
    {
        XRLiftRot = XRLift.transform.rotation.eulerAngles.x;


        int headRotDirection;
        if (XRLiftRot == 0)
        {
            headRotDirection = 0;

        }
        else if (XRLiftRot > 180)
        {

            headRotDirection = -1;
        }
        else if (XRLiftRot > 0 && XRLiftRot < 180)
        {
            headRotDirection = 1;
        }
        else
        {
            headRotDirection = 0;
        }

        rotationChange = headRotDirection * speed * Time.fixedDeltaTime;


        MoveLift();

    }
    void initializeLiftxDrive()
    {
        var xDrive = articulationChain[liftJointIndex].xDrive;
        xDrive.stiffness = stiffness;
        xDrive.damping = damping;
        xDrive.forceLimit = forceLimit;
        articulationChain[liftJointIndex].xDrive = xDrive;
    }
    void MoveLift()
    {

        var currentDrive = articulationChain[liftJointIndex].xDrive;

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
            if (XRLiftRot >= 0 && XRLiftRot < 180)
            {
                currentDrive.target = XRLiftRot;

            }
            else
            {
                currentDrive.target = -(360 - XRLiftRot);

            }

        }


        articulationChain[liftJointIndex].xDrive = currentDrive;
        liftRotPublisher.publishLiftRot(currentDrive.target);

    }

}
