using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class MeshMaterialManager : MonoBehaviour
{
    [Tooltip("Parent of the mesh to change the materials")]
    public GameObject MeshToChangeMaterial;

    [Tooltip("A copy of the original meshes with the original material")]
    public GameObject DefaultMesh;

    [Tooltip("Material for every mesh renderer in this linac section")]
    public Material GlobalMaterial;

    public int numChildCount = 0;


    private List<Material> defaultMaterials;
    private int MaterialUpdateInd = 0;

    private List<GameObject> PlaceHourderList;
    private List<GameObject> EscapeRouteModuleList;
    private List<GameObject> EletronicRackList;


    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
            
    }

    #region generic function

    public void GetChildRecursive(GameObject obj)
    {
        if (null == obj)
            return;
        foreach (Transform child in obj.transform)
        {
            if (null == child)
                continue;
            if (child.childCount == 0)
            {
                numChildCount += 1;
                //listOfChildren.Add(child.gameObject);
            }
            else
            {

                GetChildRecursive(child.gameObject);
            }
        }
    }

    public void SetMaterialRecursive(GameObject obj, Material mat)
    {
        // change the material of all the children in the given game object recursively
        if (null == obj)
            return;
        foreach (Transform child in obj.transform)
        {
            if (null == child)
                continue;
            if (child.TryGetComponent<Renderer>(out var meshRenderer))
            {
                // only the youngest child has the mesh rendere
                meshRenderer.sharedMaterial = mat;
            }
            if (child.childCount == 0)
                continue;
            else
            {
                SetMaterialRecursive(child.gameObject, mat);
                // keep recursion if couldn't find the renderer
                // SetMaterialRecursive(child.gameObject,mat);
            }
        }
    }
    #endregion
    #region set default materials
    public void SetBackDefaultMaterials()
    {
        GetDefaultMaterialList();
        SetDefaultMaterialRecursive(MeshToChangeMaterial);
    }
    private void SetDefaultMaterialRecursive(GameObject activeObj)
    {
        if (null == activeObj)
            return;

        foreach (Transform child in activeObj.transform)
        {
            if (null == child)
                continue;

            if (child.TryGetComponent<Renderer>(out var meshRenderer))
            {
                // only the youngest child has the mesh rendere
                meshRenderer.sharedMaterial = defaultMaterials[MaterialUpdateInd];
                MaterialUpdateInd += 1;
            }

            if (child.childCount == 0)
                continue;
            else
            {
                // keep recursion if couldn't find the renderer
                SetDefaultMaterialRecursive(child.gameObject);
            }
        }
    }

    private void GetDefaultMaterialList()
    {
        defaultMaterials = new List<Material>();
        MaterialUpdateInd = 0;
        GetDefaultMaterialRecursive(DefaultMesh);
    }

    private void GetDefaultMaterialRecursive(GameObject obj)
    {
        // get the parent cryo module game object recursively
        if (null == obj)
            return;
        foreach (Transform child in obj.transform)
        {
            if (null == child)
                continue;

            if (child.TryGetComponent<Renderer>(out var meshRenderer))
            {
                defaultMaterials.Add(meshRenderer.sharedMaterial);
            }

            if (child.childCount == 0)
                continue;
            else
            {
                GetDefaultMaterialRecursive(child.gameObject);
            }
        }
    }
    #endregion


}
