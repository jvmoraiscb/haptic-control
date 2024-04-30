using ROS2;
using UnityEngine;
using System.Collections.Generic;

// Code based on https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/main/Nav2SLAMExampleProject/Assets/Scripts/LaserScanSensor.cs
public class LaserScanPublisher : MonoBehaviour{
    [Header("LaserScan Settings")]
    [SerializeField] private string nodeName = "LaserScanPub_Unity";
    [SerializeField] private string topicName = "scan";
    [SerializeField] private string frameIdName = "laser_link";

    [Header("LaserScan Dependencies")]
    [SerializeField] private Transform lidarPose;

    [Header("LaserScan Parameters")]
    [SerializeField] private string layerName = "Scan";
    [SerializeField] private float publisherFrequency = 10f;
    [SerializeField] private float rangeMetersMin = 0;
    [SerializeField] private float rangeMetersMax = 1000;
    [SerializeField] private float scanAngleStartDegrees = 0;
    [SerializeField] private float scanAngleEndDegrees = -359;
    [SerializeField] private float scanOffsetAfterPublish = 0f;
    [SerializeField] private int numMeasurementsPerScan = 180;
    [SerializeField] private float timeBetweenMeasurementsSeconds = 0f;
    
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<sensor_msgs.msg.LaserScan> pub;
    private float currentScanAngleStart;
    private float currentScanAngleEnd;
    private double timeNextScanSeconds = -1;
    private int numMeasurementsTaken;
    private List<float> ranges = new List<float>();
    private bool isScanning = false;
    private double timeLastScanBeganSeconds = -1;

    private void Start(){
        ros2Unity = GetComponent<ROS2UnityComponent>();
        currentScanAngleStart = scanAngleStartDegrees;
        currentScanAngleEnd = scanAngleEndDegrees;
    }

    private void Update(){
        if(!ros2Unity.Ok()) return;
        if(ros2Node == null){
            ros2Node = ros2Unity.CreateNode(nodeName);
            pub = ros2Node.CreatePublisher<sensor_msgs.msg.LaserScan>(topicName);

            timeNextScanSeconds = Ros2Clock.Now + 1/publisherFrequency;
        }
        
        if (!isScanning){
            if (Ros2Clock.NowTimeInSeconds < timeNextScanSeconds) return;
            BeginScan();
        }
        var measurementsSoFar = timeBetweenMeasurementsSeconds == 0 ? numMeasurementsPerScan : 1 + Mathf.FloorToInt((float)(Ros2Clock.time - timeLastScanBeganSeconds) / timeBetweenMeasurementsSeconds);
        if (measurementsSoFar > numMeasurementsPerScan)
            measurementsSoFar = numMeasurementsPerScan;

        var yawBaseDegrees = lidarPose.rotation.eulerAngles.y;
        while (numMeasurementsTaken < measurementsSoFar){
            var t = numMeasurementsTaken / (float)numMeasurementsPerScan;
            var yawSensorDegrees = Mathf.Lerp(currentScanAngleStart, currentScanAngleEnd, t);
            var yawDegrees = yawBaseDegrees + yawSensorDegrees;
            var directionVector = Quaternion.Euler(0f, yawDegrees, 0f) * Vector3.forward;
            var measurementStart = rangeMetersMin * directionVector + lidarPose.position;
            var measurementRay = new Ray(measurementStart, directionVector);
            var layerMask = 1 << LayerMask.NameToLayer(layerName);
            var foundValidMeasurement = Physics.Raycast(measurementRay, out var hit, rangeMetersMax, layerMask);
            // Only record measurement if it's within the sensor's operating range
            if (foundValidMeasurement){
                ranges.Add(hit.distance);
            }
            else{
                ranges.Add(float.MaxValue);
            }

            // Even if Raycast didn't find a valid hit, we still count it as a measurement
            ++numMeasurementsTaken;
        }
        
        if (numMeasurementsTaken >= numMeasurementsPerScan){
            if (numMeasurementsTaken > numMeasurementsPerScan){
                Debug.LogError($"LaserScan has {numMeasurementsTaken} measurements but we expected {numMeasurementsPerScan}");
            }
            EndScan();
        }
    }

    private void BeginScan(){
        isScanning = true;
        timeLastScanBeganSeconds = Ros2Clock.Now;
        timeNextScanSeconds = timeLastScanBeganSeconds + 1/publisherFrequency;
        numMeasurementsTaken = 0;
    }

    private void EndScan(){
        if (ranges.Count == 0){
            Debug.LogWarning($"Took {numMeasurementsTaken} measurements but found no valid ranges");
        }
        else if (ranges.Count != numMeasurementsTaken || ranges.Count != numMeasurementsPerScan){
            Debug.LogWarning($"Expected {numMeasurementsPerScan} measurements. Actually took {numMeasurementsTaken}" +
                             $"and recorded {ranges.Count} ranges.");
        }

        var timestamp = new TimeStamp(Ros2Clock.time);
        // Invert the angle ranges when going from Unity to ROS
        var angleStartRos = -currentScanAngleStart * Mathf.Deg2Rad;
        var angleEndRos = -currentScanAngleEnd * Mathf.Deg2Rad;
        if (angleStartRos > angleEndRos){
            Debug.LogWarning("LaserScan was performed in a clockwise direction but ROS expects a counter-clockwise scan, flipping the ranges...");
            var temp = angleEndRos;
            angleEndRos = angleStartRos;
            angleStartRos = temp;
            ranges.Reverse();
        }

        var msg = new sensor_msgs.msg.LaserScan{
            Header = new std_msgs.msg.Header{
                Frame_id = frameIdName,
                Stamp = new builtin_interfaces.msg.Time{
                    Sec = timestamp.Seconds,
                    Nanosec = timestamp.NanoSeconds,
                }
            },
            Range_min = rangeMetersMin,
            Range_max = rangeMetersMax,
            Angle_min = angleStartRos,
            Angle_max = angleEndRos,
            Angle_increment = (angleEndRos - angleStartRos) / numMeasurementsPerScan,
            Time_increment = timeBetweenMeasurementsSeconds,
            Scan_time = 1/publisherFrequency,
            Intensities = new float[ranges.Count],
            Ranges = ranges.ToArray(),
        };
        
        pub.Publish(msg);

        numMeasurementsTaken = 0;
        ranges.Clear();
        isScanning = false;
        var now = (float)Ros2Clock.time;
        if (now > timeNextScanSeconds){
            Debug.LogWarning($"Failed to complete scan started at {timeLastScanBeganSeconds:F} before next scan was " +
                             $"scheduled to start: {timeNextScanSeconds:F}, rescheduling to now ({now:F})");
            timeNextScanSeconds = now;
        }

        currentScanAngleStart += scanOffsetAfterPublish;
        currentScanAngleEnd += scanOffsetAfterPublish;
        if (currentScanAngleStart > 360f || currentScanAngleEnd > 360f){
            currentScanAngleStart -= 360f;
            currentScanAngleEnd -= 360f;
        }
    }
}