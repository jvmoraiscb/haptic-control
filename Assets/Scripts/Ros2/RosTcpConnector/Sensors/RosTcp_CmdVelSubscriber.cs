using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RosTcp_CmdVelSubscriber : MonoBehaviour{
    [Header("CmdVel Settings")]
    [SerializeField] private string topicName = "cmd_vel";

    [Header("CmdVel Dependencies")]
    [SerializeField] private AckermannMiddleware ackermannMid;

    private ROSConnection ros;

    private void Start(){
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topicName, CmdVelCallback);
    }

    private void CmdVelCallback(TwistMsg msg){
        ackermannMid.Throttle = (float)msg.linear.x;
        ackermannMid.Steer = (float)msg.angular.z;
    }
}