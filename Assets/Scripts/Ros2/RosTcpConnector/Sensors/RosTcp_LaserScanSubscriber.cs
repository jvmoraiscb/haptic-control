using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

// Code based on https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/com.unity.robotics.visualizations/Runtime/DefaultVisualizers/Sensor/ScriptableObjects/LaserScanVisualizerSettings.cs
public class RosTcp_LaserScanSubscriber : MonoBehaviour{
    [Header("Scan Settings")]
    [SerializeField] private bool isEnabled = true;
    [SerializeField] private string topicName = "scan";
    
    [Header("Scan Dependencies")]
    [SerializeField] private Transform lidarPose;
    [SerializeField] private GameObject wallPrefab;

    private ROSConnection ros;
    private System.Collections.Generic.List<GameObject> walls;

    private void Start(){
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<LaserScanMsg>(topicName, LaserScanCallback);
        
        walls = new System.Collections.Generic.List<GameObject>();
    }

    private void LaserScanCallback(LaserScanMsg msg){
        if (!isEnabled) return;
        foreach (GameObject wall in walls){
            Destroy(wall);
        }
        walls.Clear();
        // negate the angle because ROS coordinates are right-handed, unity coordinates are left-handed
        float angle = -msg.angle_min;
        for (int i = 0; i < msg.ranges.Length; i++){
            // create a point and move it relative to the position and rotation of the lidar
            Vector3 point = Quaternion.Euler(0, lidarPose.eulerAngles.y + Mathf.Rad2Deg * angle, 0) * Vector3.forward * msg.ranges[i] + lidarPose.position;
            walls.Add(Instantiate(wallPrefab, point, new Quaternion(0, 0, 0, 1)));
            angle -= msg.angle_increment;
        }
    }
}