using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Boundary;
using Microsoft.MixedReality.Toolkit;

public class CozmoSimulationManager : MonoBehaviour, IMixedRealityBoundaryHandler
{
    public GameObject CozmoRobot;
    public GameObject Table;

    void Start()
    {
        //SetToFloorHeight();
    }

    private void OnEnable()
    {
        //CoreServices.BoundarySystem?.RegisterHandler<IMixedRealityBoundaryHandler>(this);
    }

    private void OnDisable()
    {
       // CoreServices.BoundarySystem?.UnregisterHandler<IMixedRealityBoundaryHandler>(this);
    }


    public void ToggleSimulation()
    {
        if(CozmoRobot != null && Table != null)
        {
            bool isActive = CozmoRobot.active;
            CozmoRobot.SetActive(!isActive);
            Table.SetActive(!isActive);
        }
    }

    // Update is called once per frame
    void Update()
    {
        //if (CoreServices.BoundarySystem != null)
        //{
        //    var boundarySystem = CoreServices.BoundarySystem;
        //    boundarySystem.ShowFloor = true;
        //}
        //else
        //{
        //    Debug.Log("Boundary system is null!");
        //}
        // sycrchronize transform

        if(CozmoRobot != null && Table != null)
        {
            //CozmoRobot.transform.position = Table.transform.position;
        }
    }

    //Set the invisible table and the robot to floor height based on the tracking of the headset
    public void SetToFloorHeight()
    {
        float floorHeight = (float)CoreServices.BoundarySystem.FloorHeight;
        Vector3 newPos = new Vector3(Table.transform.position.x, floorHeight, Table.transform.position.z);
        Table.transform.position = newPos;

        Debug.Log("Floor height: " + floorHeight);
        // cozmo robot will fall based on gravity

    }
    public void OnBoundaryVisualizationChanged(BoundaryEventData eventData)
    {
        Debug.Log("[BoundaryVisualizationDemo] Boundary visualization changed.");
    }
}
