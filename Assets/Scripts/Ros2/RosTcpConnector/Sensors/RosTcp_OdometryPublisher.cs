using UnityEngine;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;

public class RosTcp_OdometryPublisher : MonoBehaviour{
    [Header("Odometry Settings")]
    [SerializeField] private string topicName = "odom";
    [SerializeField] private string frameIdName = "odom";
    [SerializeField] private string childFrameIdName = "base_link";

    [Header("Odometry Dependencies")]
    [SerializeField] private AckermannMiddleware ackermannMid;

    [Header("Odometry Parameters")]
    [SerializeField] private float publisherFrequency = 10f;

    private ROSConnection ros;
    private double timeNextOdomSeconds = -1;

    private void Start(){
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>(topicName);

        timeNextOdomSeconds = Clock.Now + 1/publisherFrequency;
    }

    private void Update(){
        if(Clock.NowTimeInSeconds < timeNextOdomSeconds) return;
        var timestamp = new TimeStamp(Clock.Now);
        var position = ackermannMid.Position;
        var rotation = ackermannMid.Rotation;
        // ignore y position and x and z rotation (in unity)
        position.y = 0f;
        rotation.x = 0f;
        rotation.z = 0f;
        var msg = new OdometryMsg{
            header = new HeaderMsg{
                frame_id = frameIdName,
                stamp = new TimeMsg{
                    sec = timestamp.Seconds,
                    nanosec = timestamp.NanoSeconds
                }
            },
            child_frame_id = childFrameIdName,
            pose = new PoseWithCovarianceMsg{
                pose = new PoseMsg{
                    position = Transformations.Unity2Ros(position),
                    orientation = Transformations.Unity2Ros(rotation)
                }
            }
        };
        ros.Publish(topicName, msg);
    }
}
