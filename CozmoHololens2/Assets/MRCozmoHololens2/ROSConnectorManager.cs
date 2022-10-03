using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics;

public class ROSConnectorManager : MonoBehaviour
{
    public TouchScreenKeyboard keyboard;
    public ROSConnection rosConnection;
    public GameObject UICanvas;
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (keyboard != null && rosConnection!=null)
        {
            rosConnection.RosIPAddress = keyboard.text;
            
            // Do stuff with keyboardText
        }
    }

    public void UpdateRosIPAddr()
    {
        rosConnection.Connect();
        hideUICanvas();
        Debug.Log("reconnecting to ROS with new ip addr...");
    }

    public void ToggleUICanvas()
    {
        if(UICanvas != null)
        {
            bool isactive = UICanvas.active;
            UICanvas.SetActive(!isactive);
        }
    }
    public void activateUICanvas()
    {
        if (UICanvas != null)
        {
            UICanvas.SetActive(true);
        }
    }
    public void hideUICanvas()
    {
        if(UICanvas != null)
        {
            UICanvas?.SetActive(false);
        }
    }
   
    public void OpenSystemKeyboard()
    {
        keyboard = TouchScreenKeyboard.Open("", TouchScreenKeyboardType.Default, false, false, false, false);
    }
}
