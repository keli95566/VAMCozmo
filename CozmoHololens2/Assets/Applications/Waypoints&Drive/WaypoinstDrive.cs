using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;

public class WaypoinstDrive : MonoBehaviour
{
    public GameObject Table;
    public GameObject Menu;
    

    private bool isActive = false;
    private PointerHandler pointerHandler;


    void Start()
    {
        pointerHandler = Table.GetComponent<PointerHandler>();
        pointerHandler.enabled = isActive;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void toggleWaypointsDrive()
    {
        pointerHandler.enabled = !isActive;
        Menu.active = !isActive;
        isActive = !isActive;

    }

}
