using System.Collections;
using System.Collections.Generic;
using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using UnityEngine;


namespace Microsoft.MixedReality.Toolkit.Examples
{
    public class HololensCozmoFingerTipController : MonoBehaviour, IMixedRealityGestureHandler<Vector3>
    {
        public GameObject CozmoRobot;

        public CozmoWheelControlublisher cmdPublisher;
        public float speed = 1.5f;
        public float angularSpeed = 15f;

        public LineRenderer LineVisualizer;

        private float targetLinearSpeed;
        private float targetAngularSpeed;
        private ArticulationWheelController wheelController;

        private IMixedRealityHandJointService handJointService = null;
        private IMixedRealityDataProviderAccess dataProviderAccess = null;

        void Start()
        {
            handJointService = CoreServices.GetInputSystemDataProvider<IMixedRealityHandJointService>();

            if (handJointService == null)
            {
                Debug.LogError("Can't get IMixedRealityHandJointService.");
                return;
            }

            dataProviderAccess = CoreServices.InputSystem as IMixedRealityDataProviderAccess;
            if (dataProviderAccess == null)
            {
                Debug.LogError("Can't get IMixedRealityDataProviderAccess.");
                return;
            }


            //PointerUtils.SetHandRayPointerBehavior(PointerBehavior.AlwaysOff);
            wheelController = CozmoRobot.GetComponent<ArticulationWheelController>();

            //initialize line renderer

        }

        void Update()
        {
            var rightIndexTip = handJointService.RequestJointTransform(TrackedHandJoint.IndexTip, Handedness.Right);
            if (rightIndexTip == null)
            {
                Debug.Log("can nogt find right index tip");
            }
            LineVisualizer.positionCount += 1;
            LineVisualizer.SetPosition(LineVisualizer.positionCount - 1, rightIndexTip.position);


        }

        public void OnGestureStarted(InputEventData eventData)
        {
            Debug.Log($"OnGestureStarted [{Time.frameCount}]: {eventData.MixedRealityInputAction.Description}");

            MixedRealityInputAction action = eventData.MixedRealityInputAction;
            Debug.Log(action);
        }


        public void OnGestureUpdated(InputEventData eventData)
        {
            Debug.Log($"OnGestureUpdated [{Time.frameCount}]: {eventData.MixedRealityInputAction.Description}");

            MixedRealityInputAction action = eventData.MixedRealityInputAction;

        }

        public void OnGestureUpdated(InputEventData<Vector3> eventData)
        {
            Debug.Log($"OnGestureUpdated [{Time.frameCount}]: {eventData.MixedRealityInputAction.Description}");

            MixedRealityInputAction action = eventData.MixedRealityInputAction;

        }
        public void OnGestureCompleted(InputEventData<Vector3> eventData)
        {
            Debug.Log($"OnGestureCompleted [{Time.frameCount}]: {eventData.MixedRealityInputAction.Description}");


        }

        public void OnGestureCompleted(InputEventData eventData)
        {
            Debug.Log($"OnGestureCompleted [{Time.frameCount}]: {eventData.MixedRealityInputAction.Description}");


        }

        public void OnGestureCanceled(InputEventData eventData)
        {
            Debug.Log($"OnGestureCanceled [{Time.frameCount}]: {eventData.MixedRealityInputAction.Description}");


        }

    }
}