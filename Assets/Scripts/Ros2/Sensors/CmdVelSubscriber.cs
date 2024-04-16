using UnityEngine;
using ROS2;

public class CmdVelSubscriber : MonoBehaviour{
    [Header("CmdVel Settings")]
    [SerializeField] private string nodeName = "CmdVelSub_Unity";
    [SerializeField] private string topicName = "cmd_vel";

    [Header("CmdVel Dependencies")]
    [SerializeField] private AckermannMiddleware ackermannMid;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<geometry_msgs.msg.Twist> sub;

    private void Start(){
        ros2Unity = GetComponent<ROS2UnityComponent>();
    }

    private void Update(){
        if(!ros2Unity.Ok()) return;
        if(ros2Node == null){
            ros2Node = ros2Unity.CreateNode(nodeName);
            sub = ros2Node.CreateSubscription<geometry_msgs.msg.Twist>(topicName, msg => CmdVelCallback(msg));
        }
    }

    private void CmdVelCallback(geometry_msgs.msg.Twist msg){
        ackermannMid.Throttle = (float)msg.Linear.X;
        ackermannMid.Steer = (float)msg.Angular.Z;
    }
}