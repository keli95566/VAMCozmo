using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(MeshMaterialManager))]

public class MeshMaterialManagerEditor : Editor
{        
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        MeshMaterialManager manager = (MeshMaterialManager)target;


        if (GUILayout.Button("Find Number of Total Children"))
        {
            manager.GetChildRecursive(manager.MeshToChangeMaterial);
            Debug.Log("Total Number of children in the global module:" + manager.numChildCount);
        }
        if (GUILayout.Button("Set Global Material"))
        {
            if (manager.GlobalMaterial != null && manager.MeshToChangeMaterial != null)
            {
                manager.SetMaterialRecursive(manager.MeshToChangeMaterial, manager.GlobalMaterial);
            }
        }
        if (GUILayout.Button("Set Back to Default Material"))
        {
            if (manager.DefaultMesh != null && manager.MeshToChangeMaterial != null)
            {
                manager.SetBackDefaultMaterials();
            }
        }
    }

}
