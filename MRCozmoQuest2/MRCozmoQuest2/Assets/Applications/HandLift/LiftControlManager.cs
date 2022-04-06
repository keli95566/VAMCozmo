using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

namespace MRCozmo
{

    public class LiftControlManager : MonoBehaviour
    {

        public GameObject HandLiftController;
        public GameObject HandRuler;
        public GameObject ControlMenu;
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
            if(HandLiftController!= null && HandRuler != null && ControlMenu!=null)
            {
                bool isActive = HandLiftController.active;
                HandLiftController.active = !isActive;
                HandRuler.active = !isActive;
              //  ControlMenu.active = !isActive;

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