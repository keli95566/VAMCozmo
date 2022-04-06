using System.Collections;
using System.Collections.Generic;
using UnityEngine.UI;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class RosVideoStreamSubscriber : MonoBehaviour
{

    [SerializeField]
    ROSConnection ros;

    void Start()
    {

        ROSConnection.instance.Subscribe<ImageMsg>("/cozmo_camera/image", ImageUpdate);
    }

    void ImageUpdate(ImageMsg cozmoImg)
    {
        RawImage image = this.GetComponent<RawImage>();
       // Debug.Log("cozmo Image stream"  + cozmoImg.data);
       // Debug.Log( "height: " + cozmoImg.height + "width: " +cozmoImg.width);
       // Debug.Log("lenth" + cozmoImg.data.Length);
       // Debug.Log("first ele" + cozmoImg.data[0]);
        int height = (int)cozmoImg.height;
        int width = (int)cozmoImg.width;

        var tex = new Texture2D(width, height, TextureFormat.Alpha8, false);
        tex.LoadRawTextureData(cozmoImg.data);
        for(int i=0; i< cozmoImg.data.Length; i++)
        {
            Color c = new Color();
            c.r = cozmoImg.data[i];
            c.g = cozmoImg.data[i];
            c.b = cozmoImg.data[i];
            tex.SetPixel(i + 1, Mod((int)cozmoImg.step, i+1), c);
        }
      //  tex.LoadImage(cozmoImg.data);

        
        tex.Apply();
        image.texture = tex;

    }

    int Mod(int a, int n) => (a % n + n) % n;
}
