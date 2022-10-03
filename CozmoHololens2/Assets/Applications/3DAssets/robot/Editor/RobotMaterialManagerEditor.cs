using UnityEditor;
using UnityEngine;


namespace MRCozmo
{

    [CustomEditor(typeof(RobotMaterialManager))]

    public class RobotMaterialManagerEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            RobotMaterialManager manager = (RobotMaterialManager)target;


            if (GUILayout.Button("Find Number of Total Children"))
            {
                manager.GetChildRecursive(manager.RobotModel);
                Debug.Log("Total Number of children in the global module:" + manager.numChildCount);
            }
            if (GUILayout.Button("Set Global Material"))
            {
                if (manager.GlobalMaterial != null && manager.RobotModel != null)
                {
                    manager.SetMaterialRecursive(manager.RobotModel, manager.GlobalMaterial);
                }
            }
            if (GUILayout.Button("Set Back to Default Material"))
            {
                if (manager.DefaultRobotModel != null && manager.RobotModel != null)
                {
                    manager.SetBackDefaultMaterials();
                }
            }

        }
    }

}
