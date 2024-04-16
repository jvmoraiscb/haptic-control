using ROS2;
using UnityEngine;

public class CmdVelPublisher : MonoBehaviour{
    [Header("CmdVel Settings")]
    [SerializeField] private string nodeName = "CmdVelPub_Unity";
    [SerializeField] private string topicName = "cmd_vel";

    [Header("CmdVel Dependencies")]
    [SerializeField] private AckermannMiddleware ackermannMid;

    [Header("Cmdvel Parameters")]
    [SerializeField] private float publisherFrequency = 10f;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<geometry_msgs.msg.Twist> pub;
    private double timeNextPublishInSeconds;

    private void Start(){
        ros2Unity = GetComponent<ROS2UnityComponent>();
    }

    private void Update(){
        if(!ros2Unity.Ok()) return;
        if(ros2Node == null){
            ros2Node = ros2Unity.CreateNode(nodeName);
            pub = ros2Node.CreatePublisher<geometry_msgs.msg.Twist>(topicName);

            timeNextPublishInSeconds = Ros2Clock.Now + 1/publisherFrequency;
        }

        if (Ros2Clock.NowTimeInSeconds < timeNextPublishInSeconds) return;
        var msg = new geometry_msgs.msg.Twist{
            Linear = new geometry_msgs.msg.Vector3{
                X = ackermannMid.Throttle,
                Y = 0f,
                Z = 0f
            },
            Angular = new geometry_msgs.msg.Vector3{
                X = 0f,
                Y = 0f,
                Z = ackermannMid.Steer
            }
        };
        pub.Publish(msg);
        timeNextPublishInSeconds = Ros2Clock.Now + 1/publisherFrequency;
    }
}