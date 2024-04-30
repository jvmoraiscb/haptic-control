using UnityEngine;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;

public class RosTcp_OdometrySubscriber : MonoBehaviour{
    [Header("Odometry Settings")]
    [SerializeField] private string topicName = "odom";

    [Header("Odometry Dependencies")]
    [SerializeField] private AckermannMiddleware ackermannMid;

    private ROSConnection ros;

    private void Start(){
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<OdometryMsg>(topicName, OdometryCallback);
    }

    private void OdometryCallback(OdometryMsg msg){
        // convert to unity system
        var position = Transformations.Ros2Unity(msg.pose.pose.position);
        var rotation = Transformations.Ros2Unity(msg.pose.pose.orientation);
        // ignore y position and x and z rotation
        position.y = 0f;
        rotation.x = 0f;
        rotation.z = 0f;
        ackermannMid.Position = position;
        ackermannMid.Rotation = rotation;
    }
}