using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CozmoControllerManager : MonoBehaviour
{
    public GameObject XRHeadController;
    public GameObject OneHandLiftController;
    public GameObject OneHandRuler;

    void Start()
    {
       
    }

    // Update is called once per frame
    void Update()
    {
        
    }


    public void activateOneHandRuler()
    {
        OneHandRuler.SetActive(true);
        OneHandLiftController.SetActive(true);
    }

    public void deActivateOneHandRuler()
    {
        OneHandRuler.SetActive(false);
        OneHandLiftController.SetActive(false);

    }

    public void activateXRHeadController()
    {
        OneHandRuler.SetActive(true);
    }

    public void deActivateXRHeadController()
    {
        OneHandRuler.SetActive(false);
    }



}
