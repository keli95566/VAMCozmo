using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MRCozmo.Math;
using MRCozmo;
public class KeyboardRobotMovementController : MonoBehaviour
{

    public GameObject CozmoRobot;
    public Transform BaseJointTransform;

    public CozmoWheelControlublisher cmdPublisher;
    public GameObject PosIndicatorPrefab;

    public float speed = 1.5f;
    public float angularSpeed = 15f;
    private float targetLinearSpeed;
    private float targetAngularSpeed;

    private float maxLinearSpeed = 0.220f;   // max linear speed [mm/s]    
    private float maxAngularSpeed = 9.82f;    // max angular speed [rad/s]

    private ArticulationWheelController wheelController;
    private bool hasFinished = false;
    private bool isRunning = false;
    private float startRot = 0;
    // Start is called before the first frame update
    void Start()
    {
        wheelController = CozmoRobot.GetComponent<ArticulationWheelController>();
    }

    // Update is called once per frame
    void Update()
    {
        targetLinearSpeed = Input.GetAxisRaw("Vertical") * speed;
        targetAngularSpeed = -Input.GetAxisRaw("Horizontal") * angularSpeed;

        
    }

    private void FixedUpdate()
    {

        // for keyboard interaction in editors
        if (Input.GetKey(KeyCode.UpArrow) || Input.GetKey(KeyCode.DownArrow))
        {
            wheelController.setRobotVelocity(targetLinearSpeed, 0);

        }

        if (Input.GetKey(KeyCode.LeftArrow) || Input.GetKey(KeyCode.RightArrow))
        {

            wheelController.setRobotVelocity(targetLinearSpeed, targetAngularSpeed);

        }
        // motion unit tests....

        if (Input.GetKey(KeyCode.S))
        {
            // stop moving
            Debug.Log("Stop all movements..");
            wheelController.setRobotVelocity(0, 0);
        }

        if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            float a = Random.Range(0, 180);
            Debug.Log("Turn left at random angle: " + a);
            StartCoroutine(wheelController.TurnInPlace(a, 1));
            Debug.Log(BaseJointTransform.forward);

        }

        if (Input.GetKeyDown(KeyCode.Alpha2))
        {
            float a = Random.Range(0, 180);

            Debug.Log("Turn right at random angle: " + a);
            StartCoroutine(wheelController.TurnInPlace(a, -1));

        }

        if (Input.GetKeyDown(KeyCode.Alpha3))
        {
            Debug.Log("Testing driving straight for 0.5 m forward..");
            StartCoroutine(wheelController.DriveStraight(0.5f, 1));

        }

        if (Input.GetKeyDown(KeyCode.Alpha4))
        {
            Debug.Log("Testing driving straight for 0.5 m backwards..");
            StartCoroutine(wheelController.DriveStraight(0.5f, -1));
        }

        if (Input.GetKeyDown(KeyCode.Alpha5))
        {
            StartCoroutine(TurnAndDrive());
        }

        if (Input.GetKeyDown(KeyCode.Alpha6))
        {
            Debug.Log("Testing prediction functionalities");
            //wheelController.current

            wheelController.currentAngularSpeed = 5;
            wheelController.currentLinearSpeed = 0.2f;
            wheelController.isDriving = true;
            float waitTime = 3.0f;
            Vector3 futurePos = wheelController.predictFuturePosition(waitTime);
            Vector3 ObjPos = new Vector3(futurePos.x, wheelController.BaseJointTransform.position.y, futurePos.y);
            Vector3 direction = new Vector3(Mathf.Sin(Mathf.Deg2Rad * futurePos.z), 0, Mathf.Cos(Mathf.Deg2Rad * futurePos.z));
            Vector3 point_C = ObjPos + (direction.normalized * 0.5f);


            Instantiate(PosIndicatorPrefab, ObjPos, Quaternion.AngleAxis(futurePos.z, Vector3.up));
            StartCoroutine(WaitAndStop(waitTime));
            Debug.Log("Predicted future pos: " + futurePos);


        }
        if (Input.GetKeyDown(KeyCode.Alpha7))
        {

            float waitTime = 3.0f;
            Vector3 futurePos = wheelController.predictFuturePosition(waitTime);
            Vector3 ObjPos = new Vector3(futurePos.x, wheelController.BaseJointTransform.position.y, futurePos.y);

            Instantiate(PosIndicatorPrefab, ObjPos, Quaternion.AngleAxis(futurePos.z, Vector3.up));
            StartCoroutine(Wait(waitTime));
            Debug.Log("Predicted future pos: " + futurePos);


        }
    }
    private IEnumerator Wait(float waitTime)
    {
        yield return new WaitForSeconds(waitTime);
    }
    private IEnumerator WaitAndStop(float waitTime)
    {
        yield return new WaitForSeconds(waitTime);

        wheelController.isDriving = false;
        wheelController.setRobotVelocity(0, 0);
        Debug.Log("Actual end pos: " + wheelController.BaseJointTransform.position + " rotation: " + RobotMathUtils.WrapAngle(wheelController.BaseJointTransform.rotation.eulerAngles.y));
    }
    private IEnumerator TurnAndDrive()
    {
        // execute action one after another

        yield return StartCoroutine(wheelController.TurnInPlace(45, -1));
        yield return StartCoroutine(wheelController.DriveStraight(0.5f, 1));

    }
}
