using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JoystickDrive : MonoBehaviour
{
    // Start is called before the first frame update
    public GameObject DriveController;
    private bool isActive = false;

    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void ToggleJoystickDrive()
    {
        DriveController.active = !isActive;
        isActive = !isActive;
    }
}
