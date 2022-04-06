using UnityEngine;
using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;

namespace MRCozmo
{
    public class HandDrawOnPlane : MonoBehaviour, IMixedRealityTouchHandler
    {
        // draw based on the index finger

        public LineRenderer LineVisualizer;

        // for visual cue of drawing
        private Renderer TargetRenderer;
        private Color originalColor;
        private Color highlightedColor;
        protected float duration = 1.5f;
        protected float t = 0;

        // for index finger position tracking
        private IMixedRealityHandJointService handJointService = null;
        private IMixedRealityDataProviderAccess dataProviderAccess = null;

        // finter draw control
        private bool isDrawing;
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

            TargetRenderer = GetComponentInChildren<Renderer>();
            if ((TargetRenderer != null) && (TargetRenderer.sharedMaterial != null))
            {
                originalColor = TargetRenderer.sharedMaterial.color;
                highlightedColor = new Color(originalColor.r + 0.2f, originalColor.g + 0.2f, originalColor.b + 0.2f);
            }
        }

        // Update is called once per frame
        void Update()
        {
            if (isDrawing)
            {
                var rightIndexTip = handJointService.RequestJointTransform(TrackedHandJoint.IndexTip, Handedness.Right);
                if (rightIndexTip == null)
                {
                    Debug.Log("can nogt find right index tip");
                }
                LineVisualizer.positionCount += 1;
                LineVisualizer.SetPosition(LineVisualizer.positionCount - 1, rightIndexTip.position);

            }
        }

        void IMixedRealityTouchHandler.OnTouchCompleted(HandTrackingInputEventData eventData)
        {
           // Debug.Log("Touch location: " + eventData.InputData.x + " " + eventData.InputData.y + "  " + eventData.InputData.z + "  ");
            isDrawing = false;
            if ((TargetRenderer != null) && (TargetRenderer.material != null))
            {
                TargetRenderer.material.color = originalColor;
            }
        }

        void IMixedRealityTouchHandler.OnTouchUpdated(HandTrackingInputEventData eventData)
        {

            // Debug.Log("Touch location: " + eventData.InputData.x + " " + eventData.InputData.y + "  " + eventData.InputData.z + "  ");
            if ((TargetRenderer != null) && (TargetRenderer.material != null))
            {
                TargetRenderer.material.color = Color.Lerp(Color.green, Color.red, t);
                t = Mathf.PingPong(Time.time, duration) / duration;
            }
        }

        void IMixedRealityTouchHandler.OnTouchStarted(HandTrackingInputEventData eventData)
        {
            
            if (TargetRenderer != null)
            {
                TargetRenderer.sharedMaterial.color = Color.Lerp(originalColor, highlightedColor, 2.0f);
            }

            isDrawing = true;
            LineVisualizer.positionCount = 0;
        }


    }
}