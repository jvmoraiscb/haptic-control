using UnityEngine;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RosTcp_CmdVelPublisher : MonoBehaviour{
    [Header("CmdVel Settings")]
    [SerializeField] private string topicName = "cmd_vel";

    [Header("CmdVel Dependencies")]
    [SerializeField] private AckermannMiddleware ackermannMid;

    [Header("Cmdvel Parameters")]
    [SerializeField] private float publisherFrequency = 10f;

    private ROSConnection ros;
    private double timeNextCmdVelSeconds = -1;

    private void Start(){
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
        
        timeNextCmdVelSeconds = Clock.Now + 1/publisherFrequency;
    }

    private void Update(){
        if (Clock.NowTimeInSeconds < timeNextCmdVelSeconds) return;
        var msg = new TwistMsg{
            linear = new Vector3Msg{
                x = ackermannMid.Throttle,
                y = 0f,
                z = 0f
            },
            angular = new Vector3Msg{
                x = 0f,
                y = 0f,
                z = ackermannMid.Steer
            }
        };
        ros.Publish(topicName, msg);
    }
}