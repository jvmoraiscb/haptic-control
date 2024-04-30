using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.BuiltinInterfaces;

// Code based on https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/main/Nav2SLAMExampleProject/Assets/Scripts/LaserScanSensor.cs
public class RosTcp_LaserScanPublisher : MonoBehaviour{
    [Header("LaserScan Settings")]
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
    
    private ROSConnection ros;
    private float currentScanAngleStart;
    private float currentScanAngleEnd;
    private double timeNextScanSeconds = -1;
    private int numMeasurementsTaken;
    private List<float> ranges = new List<float>();
    private bool isScanning = false;
    private double timeLastScanBeganSeconds = -1;

    private void Start(){
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topicName);
        
        timeNextScanSeconds = Clock.Now + 1/publisherFrequency;
        
        currentScanAngleStart = scanAngleStartDegrees;
        currentScanAngleEnd = scanAngleEndDegrees;
    }

    private void Update(){
        if (!isScanning){
            if (Clock.NowTimeInSeconds < timeNextScanSeconds) return;
            BeginScan();
        }
        var measurementsSoFar = timeBetweenMeasurementsSeconds == 0 ? numMeasurementsPerScan : 1 + Mathf.FloorToInt((float)(Clock.time - timeLastScanBeganSeconds) / timeBetweenMeasurementsSeconds);
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
        timeLastScanBeganSeconds = Clock.Now;
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

        var timestamp = new TimeStamp(Clock.time);
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

        var msg = new LaserScanMsg{
            header = new HeaderMsg{
                frame_id = frameIdName,
                stamp = new TimeMsg{
                    sec = timestamp.Seconds,
                    nanosec = timestamp.NanoSeconds,
                }
            },
            range_min = rangeMetersMin,
            range_max = rangeMetersMax,
            angle_min = angleStartRos,
            angle_max = angleEndRos,
            angle_increment = (angleEndRos - angleStartRos) / numMeasurementsPerScan,
            time_increment = timeBetweenMeasurementsSeconds,
            scan_time = 1/publisherFrequency,
            intensities = new float[ranges.Count],
            ranges = ranges.ToArray(),
        };
        
        ros.Publish(topicName, msg);

        numMeasurementsTaken = 0;
        ranges.Clear();
        isScanning = false;
        var now = (float)Clock.time;
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