using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HeadControllerManager : MonoBehaviour
{
    public GameObject HeadMovementController;

    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void ToggleHeadController()
    {
        if (HeadMovementController != null)
        {
            bool isActive = HeadMovementController.active;
            Debug.Log(isActive);
            HeadMovementController.SetActive(!isActive);

        }
    }
}
