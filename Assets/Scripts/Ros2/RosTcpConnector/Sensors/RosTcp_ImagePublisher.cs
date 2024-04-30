using UnityEngine;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.BuiltinInterfaces;

// Code based on https://github.com/Unity-Technologies/ROS-TCP-Connector/issues/223
public class RosTcp_ImagePublisher : MonoBehaviour{
    [Header("Image Settings")]
    [SerializeField] private string topicName = "image_raw";
    [SerializeField] private string frameIdName = "camera_link";

    [Header("Image Dependencies")]
    [SerializeField] private Camera cam;

    [Header("Image Parameters")]
    [SerializeField] private float publisherFrequency = 10f;

    private ROSConnection ros;
    private double timeNextImageSeconds = -1;
    private Texture2D texture = null;
    void Start(){
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);
        
        timeNextImageSeconds = Clock.Now + 1/publisherFrequency;
        texture = new Texture2D(cam.targetTexture.width, cam.targetTexture.height);
    }

    void Update(){
        if (Clock.NowTimeInSeconds < timeNextImageSeconds) return;
        RenderTexture.active = cam.targetTexture;
        cam.Render();
        
        // Copy the pixels from the GPU into a texture so we can work with them
        // For more efficiency you should reuse this texture, instead of creating and disposing them every time
        // Texture2D camText = new Texture2D(cam.targetTexture.width, cam.targetTexture.height);
        texture.ReadPixels(new Rect(0, 0, cam.targetTexture.width, cam.targetTexture.height), 0, 0);
        texture.Apply();
        
        // Encode the texture as an ImageMsg, and send to ROS
        var timestamp = new TimeStamp(Clock.Now);
        var header = new HeaderMsg{
            frame_id = frameIdName,
            stamp = new TimeMsg{
                sec = timestamp.Seconds,
                nanosec = timestamp.NanoSeconds,
            }
        };
        var msg = texture.ToImageMsg(header);

        ros.Publish(topicName, msg);
    }
}
