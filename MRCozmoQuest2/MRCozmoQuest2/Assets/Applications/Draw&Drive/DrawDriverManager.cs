using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.UI;


namespace MRCozmo
{

    public class DrawDriverManager : MonoBehaviour
    {
        public GameObject ControlButton;
        public GameObject Follower;
        // Start is called before the first frame update

        bool isActive = false;
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {

        }

        public void ToggleDriver()
        {

            //   Table.active = !isActive;
            ControlButton.active = !isActive;
            Follower.active = !isActive;
            isActive = !isActive;
        }


        public void ToggleFix()
        {
            if(ControlButton != null && ControlButton.GetComponent<ObjectManipulator>()!= null)
            {
                ObjectManipulator a = ControlButton.GetComponent<ObjectManipulator>();
                bool isEnabled = a.enabled;
                a.enabled = !isEnabled;

                Debug.Log("Table Fix : " + ControlButton.active);
            }
        }

        
        public void ResetPath() {

        
        }
        
    }
}