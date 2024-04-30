using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.Core{
    // Code based on https://github.com/RobotecAI/ros2-for-unity/blob/develop/src/Ros2ForUnity/Scripts/Transformations.cs
    public static class Transformations{
        public static Vector3 Ros2Unity(this PointMsg msg){
            return new Vector3(-(float)msg.y, (float)msg.z, (float)msg.x);
        }

        public static PointMsg Unity2Ros(this Vector3 vector3){
            return new PointMsg(vector3.z, -vector3.x, vector3.y);
        }

        public static Quaternion Ros2Unity(this QuaternionMsg msg){
            return new Quaternion((float)msg.y, (float)-msg.z, (float)-msg.x, (float)msg.w);
        }

        public static QuaternionMsg Unity2Ros(this Quaternion quaternion){
            return new QuaternionMsg(-quaternion.z, quaternion.x, -quaternion.y, quaternion.w);
        }
    }
}