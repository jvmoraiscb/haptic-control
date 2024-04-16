using ROS2;
using UnityEngine;

public class OdometryPublisher : MonoBehaviour{
    [Header("Odometry Settings")]
    [SerializeField] private string nodeName = "OdometryPub_Unity";
    [SerializeField] private string topicName = "odom";
    [SerializeField] private string frameIdName = "odom";
    [SerializeField] private string childFrameIdName = "base_link";

    [Header("Odometry Dependencies")]
    [SerializeField] private AckermannMiddleware ackermannMid;

    [Header("Odometry Parameters")]
    [SerializeField] private float publisherFrequency = 10f;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<nav_msgs.msg.Odometry> pub;
    private double timeNextPublishInSeconds;

    private void Start(){
        ros2Unity = GetComponent<ROS2UnityComponent>();
    }

    private void Update(){
        if(!ros2Unity.Ok()) return;
        if(ros2Node == null){
            ros2Node = ros2Unity.CreateNode(nodeName);
            pub = ros2Node.CreatePublisher<nav_msgs.msg.Odometry>(topicName);

            timeNextPublishInSeconds = Ros2Clock.Now + 1/publisherFrequency;
        }

        if(Ros2Clock.NowTimeInSeconds < timeNextPublishInSeconds) return;
        var timestamp = new TimeStamp(Ros2Clock.Now);
        var position = ackermannMid.Position;
        var rotation = ackermannMid.Rotation;
        // ignore y position and x and z rotation (in unity)
        position.y = 0f;
        rotation.x = 0f;
        rotation.z = 0f;
        var msg = new nav_msgs.msg.Odometry{
            Header = new std_msgs.msg.Header{
                Frame_id = frameIdName,
                Stamp = new builtin_interfaces.msg.Time{
                    Sec = timestamp.Seconds,
                    Nanosec = timestamp.NanoSeconds
                }
            },
            Child_frame_id = childFrameIdName,
            Pose = new geometry_msgs.msg.PoseWithCovariance{
                Pose = new geometry_msgs.msg.Pose{
                    Position = new geometry_msgs.msg.Point {
                        X = Transformations.Unity2Ros(position).x,
                        Y = Transformations.Unity2Ros(position).y,
                        Z = Transformations.Unity2Ros(position).z
                    },
                    Orientation = new geometry_msgs.msg.Quaternion {
                        X = Transformations.Unity2Ros(rotation).x,
                        Y = Transformations.Unity2Ros(rotation).y,
                        Z = Transformations.Unity2Ros(rotation).z,
                        W = Transformations.Unity2Ros(rotation).w
                    }
                }
            }
        };
        pub.Publish(msg);
        timeNextPublishInSeconds = Ros2Clock.Now + 1/publisherFrequency;
    }
}
