using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace MRCozmo
{
    public class RobotMaterialManager : MonoBehaviour
    {
        [Tooltip("Robot model as given by the CAD data")]
        public GameObject RobotModel;

        [Tooltip("The default robot model to parse and save the material")]
        public GameObject DefaultRobotModel;

        [Tooltip("Material for every mesh renderer in the robot game object")]
        public Material GlobalMaterial;

        public int numChildCount = 0;
        private List<Material> defaultMaterials;
        private int MaterialUpdateInd = 0;

        private List<GameObject> Wheels;

        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {

        }
        #region
        public void GetChildRecursive(GameObject obj)
        {
            // get the parent cryo module game object recursively
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
            SetDefaultMaterialRecursive(DefaultRobotModel);
        }

        private void GetDefaultMaterialList()
        {
            defaultMaterials = new List<Material>();
            MaterialUpdateInd = 0;
            GetDefaultMaterialRecursive(DefaultRobotModel);
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
        #endregion

        #region mischellenous
        public void GetEletronicRacksObjects(GameObject obj)
        {
            // get the parent cryo module game object recursively
            if (null == obj)
                return;
            foreach (Transform child in obj.transform)
            {
                if (null == child || child.childCount == 0)
                    continue;

                else if (child.name.Contains("E-Rack-Maint"))
                {
                    Wheels.Add(child.gameObject);
                }
                else
                {
                    GetEletronicRacksObjects(child.gameObject);
                }
            }
        }
        #endregion
    }

}
