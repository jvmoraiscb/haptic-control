using ROS2;
using UnityEngine;

public class OdometrySubscriber : MonoBehaviour{
    [Header("Odometry Settings")]
    [SerializeField] private string nodeName = "OdometrySub_Unity";
    [SerializeField] private string topicName = "odom";

    [Header("Odometry Dependencies")]
    [SerializeField] private AckermannMiddleware ackermannMid;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<nav_msgs.msg.Odometry> sub;

    private void Start(){
        ros2Unity = GetComponent<ROS2UnityComponent>();
    }

    private void Update(){
        if(!ros2Unity.Ok()) return;
        if(ros2Node == null){
            ros2Node = ros2Unity.CreateNode(nodeName);
            sub = ros2Node.CreateSubscription<nav_msgs.msg.Odometry>(topicName, msg => OdometryCallback(msg));
        }
    }

    private void OdometryCallback(nav_msgs.msg.Odometry msg){
        // convert to unity system
        var position = Transformations.Ros2Unity(new Vector3 {
            x = (float)msg.Pose.Pose.Position.X,
            y = (float)msg.Pose.Pose.Position.Y,
            z = (float)msg.Pose.Pose.Position.Z,
        });
        var rotation = Transformations.Ros2Unity(new Quaternion {
            x = (float)msg.Pose.Pose.Orientation.X,
            y = (float)msg.Pose.Pose.Orientation.Y,
            z = (float)msg.Pose.Pose.Orientation.Z,
            w = (float)msg.Pose.Pose.Orientation.W,
        });
        // ignore y position and x and z rotation (in unity)
        position.y = 0f;
        rotation.x = 0f;
        rotation.z = 0f;
        ackermannMid.Position = position;
        ackermannMid.Rotation = rotation;
    }
}