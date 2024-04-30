using ROS2;
using UnityEngine;

// Code based on https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/com.unity.robotics.visualizations/Runtime/DefaultVisualizers/Sensor/ScriptableObjects/LaserScanVisualizerSettings.cs
public class LaserScanSubscriber : MonoBehaviour{
    [Header("LaserScan Settings")]
    [SerializeField] private string nodeName = "LaserScanSub_Unity";
    [SerializeField] private string topicName = "scan";
    
    [Header("LaserScan Dependencies")]
    [SerializeField] private Transform lidarPose;
    [SerializeField] private GameObject wallPrefab;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<sensor_msgs.msg.LaserScan> sub;
    private System.Collections.Generic.List<GameObject> walls;
    private sensor_msgs.msg.LaserScan msg;

    private void Start(){
        ros2Unity = GetComponent<ROS2UnityComponent>();
        walls = new System.Collections.Generic.List<GameObject>();
    }

    private void Update(){
        if(!ros2Unity.Ok()) return;
        if(ros2Node == null){
            ros2Node = ros2Unity.CreateNode(nodeName);
            sub = ros2Node.CreateSubscription<sensor_msgs.msg.LaserScan>(topicName, msg => LaserScanCallback(msg));
        }
        
        if(msg != null){
            foreach (GameObject wall in walls){
                Destroy(wall);
            }
            walls.Clear();
            // negate the angle because ROS coordinates are right-handed, unity coordinates are left-handed
            float angle = -msg.Angle_min;
            for (int i = 0; i < msg.Ranges.Length; i++){
                // create a point and move it relative to the position and rotation of the lidar
                Vector3 point = Quaternion.Euler(0, lidarPose.eulerAngles.y + Mathf.Rad2Deg * angle, 0) * Vector3.forward * msg.Ranges[i] + lidarPose.position;
                walls.Add(Instantiate(wallPrefab, point, new Quaternion(0, 0, 0, 1)));
                angle -= msg.Angle_increment;
            }
        }
    }

    private void LaserScanCallback(sensor_msgs.msg.LaserScan msg){
        this.msg = msg;
    }
}