//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.CozmoDemo
{
    [Serializable]
    public class PosRotMsg : Message
    {
        public const string k_RosMessageName = "cozmo_demo_msgs/PosRot";
        public override string RosMessageName => k_RosMessageName;

        public float pos_x;
        public float pos_y;
        public float pos_z;
        public float rot_x;
        public float rot_y;
        public float rot_z;
        public float rot_w;

        public PosRotMsg()
        {
            this.pos_x = 0.0f;
            this.pos_y = 0.0f;
            this.pos_z = 0.0f;
            this.rot_x = 0.0f;
            this.rot_y = 0.0f;
            this.rot_z = 0.0f;
            this.rot_w = 0.0f;
        }

        public PosRotMsg(float pos_x, float pos_y, float pos_z, float rot_x, float rot_y, float rot_z, float rot_w)
        {
            this.pos_x = pos_x;
            this.pos_y = pos_y;
            this.pos_z = pos_z;
            this.rot_x = rot_x;
            this.rot_y = rot_y;
            this.rot_z = rot_z;
            this.rot_w = rot_w;
        }

        public static PosRotMsg Deserialize(MessageDeserializer deserializer) => new PosRotMsg(deserializer);

        private PosRotMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.pos_x);
            deserializer.Read(out this.pos_y);
            deserializer.Read(out this.pos_z);
            deserializer.Read(out this.rot_x);
            deserializer.Read(out this.rot_y);
            deserializer.Read(out this.rot_z);
            deserializer.Read(out this.rot_w);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.pos_x);
            serializer.Write(this.pos_y);
            serializer.Write(this.pos_z);
            serializer.Write(this.rot_x);
            serializer.Write(this.rot_y);
            serializer.Write(this.rot_z);
            serializer.Write(this.rot_w);
        }

        public override string ToString()
        {
            return "PosRotMsg: " +
            "\npos_x: " + pos_x.ToString() +
            "\npos_y: " + pos_y.ToString() +
            "\npos_z: " + pos_z.ToString() +
            "\nrot_x: " + rot_x.ToString() +
            "\nrot_y: " + rot_y.ToString() +
            "\nrot_z: " + rot_z.ToString() +
            "\nrot_w: " + rot_w.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}