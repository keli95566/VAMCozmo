using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

namespace MRCozmo
{

    public class LiftControlManager : MonoBehaviour
    {

        public GameObject HandLiftController;
        // Start is called before the first frame update

        private bool publish = false;

        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {

        }

        public void ToggleHandLiftController()
        {
            if(HandLiftController!= null )
            {
                bool isActive = HandLiftController.active;
               // Debug.Log(isActive);
                HandLiftController.SetActive(!isActive);

            }
        }

        public void TogglePublish()
        {
            if (HandLiftController.GetComponent<ROSConnection>())
            {
                ROSConnection ros = HandLiftController.GetComponent<ROSConnection>();
                if (ros.HasConnectionThread)
                {
                   
                    HandLiftController.GetComponent<CozmoHololensHandLiftController>().publish = !publish;
                    publish = !publish;
                }
                else
                {
                    Debug.Log("ROS Not Connected! Can't publish");
                }
           
            }
        }

        public void StopPublish()
        {
            HandLiftController.GetComponent<CozmoHololensHandLiftController>().publish = false;
        }
    }
}