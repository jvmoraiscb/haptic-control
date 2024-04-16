using ROS2;
using System;
using UnityEngine;

// Code based on https://github.com/Unity-Technologies/ROS-TCP-Connector/issues/223
public class ImagePublisher : MonoBehaviour{
    [Header("Image Settings")]
    [SerializeField] private string nodeName = "ImagePub_Unity";
    [SerializeField] private string topicName = "image_raw";
    [SerializeField] private string frameIdName = "camera_link";

    [Header("Image Dependencies")]
    [SerializeField] private Camera cam;

    [Header("Image Parameters")]
    [SerializeField] private float publisherFrequency = 10f;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<sensor_msgs.msg.Image> pub;
    private double timeNextPublishInSeconds;
    private Texture2D texture = null;
    void Start(){
        ros2Unity = GetComponent<ROS2UnityComponent>();
        texture = new Texture2D(cam.targetTexture.width, cam.targetTexture.height);
    }

    void Update(){
        if(!ros2Unity.Ok()) return;
        if(ros2Node == null){
            ros2Node = ros2Unity.CreateNode(nodeName);
            pub = ros2Node.CreatePublisher<sensor_msgs.msg.Image>(topicName);

            timeNextPublishInSeconds = Ros2Clock.Now + 1/publisherFrequency;
        }

        if (Ros2Clock.NowTimeInSeconds < timeNextPublishInSeconds) return;
        RenderTexture.active = cam.targetTexture;
        cam.Render();
        
        // Copy the pixels from the GPU into a texture so we can work with them
        // For more efficiency you should reuse this texture, instead of creating and disposing them every time
        // Texture2D camText = new Texture2D(cam.targetTexture.width, cam.targetTexture.height);
        texture.ReadPixels(new Rect(0, 0, cam.targetTexture.width, cam.targetTexture.height), 0, 0);
        texture.Apply();
        
        // Encode the texture as an ImageMsg, and send to ROS
        var timestamp = new TimeStamp(Ros2Clock.Now);
        var header = new std_msgs.msg.Header{
            Frame_id = frameIdName,
            Stamp = new builtin_interfaces.msg.Time{
                Sec = timestamp.Seconds,
                Nanosec = timestamp.NanoSeconds
            }
        };
        var msg = ToImageMsg(texture, header);


        pub.Publish(msg);
        timeNextPublishInSeconds = Ros2Clock.Now + 1/publisherFrequency;
    }

    public static sensor_msgs.msg.Image ToImageMsg(Texture2D tex, std_msgs.msg.Header header){
            byte[] data;
            string encoding;
            int step;
            switch (tex.format){
                case TextureFormat.RGB24:
                    data = new byte[tex.width * tex.height * 3];
                    tex.GetPixelData<byte>(0).CopyTo(data);
                    encoding = "rgb8";
                    step = 3 * tex.width;
                    data = ReverseInBlocks(data, tex.width * 3, tex.height);
                    break;
                case TextureFormat.RGBA32:
                    data = new byte[tex.width * tex.height * 4];
                    tex.GetPixelData<byte>(0).CopyTo(data);
                    encoding = "rgba8";
                    step = 4 * tex.width;
                    data = ReverseInBlocks(data, tex.width * 4, tex.height);
                    break;
                case TextureFormat.R8:
                    data = new byte[tex.width * tex.height];
                    tex.GetPixelData<byte>(0).CopyTo(data);
                    encoding = "8UC1";
                    step = 1 * tex.width;
                    data = ReverseInBlocks(data, tex.width, tex.height);
                    break;
                case TextureFormat.R16:
                    data = new byte[tex.width * tex.height * 2];
                    tex.GetPixelData<byte>(0).CopyTo(data);
                    encoding = "16UC1";
                    step = 2 * tex.width;
                    data = ReverseInBlocks(data, tex.width * 2, tex.height);
                    break;
                default:
                    Color32[] pixels = tex.GetPixels32();
                    data = new byte[pixels.Length * 4];
                    // although this is painfully slow, it does work... Surely there's a better way
                    int writeIdx = 0;
                    for (int Idx = 0; Idx < pixels.Length; ++Idx)
                    {
                        Color32 p = pixels[Idx];
                        data[writeIdx] = p.r;
                        data[writeIdx + 1] = p.g;
                        data[writeIdx + 2] = p.b;
                        data[writeIdx + 3] = p.a;
                        writeIdx += 4;
                    }
                    data = ReverseInBlocks(data, tex.width * 4, tex.height);
                    encoding = "rgba8";
                    step = 4 * tex.width;
                    break;
            }
            return new sensor_msgs.msg.Image{
                Header = header, 
                Height = (uint)tex.height,
                Width = (uint)tex.width,
                Encoding = encoding,
                Is_bigendian = 0,
                Step = (uint)step,
                Data = data
            };
    }
    static byte[] s_ScratchSpace;
        static byte[] ReverseInBlocks(byte[] array, int blockSize, int numBlocks)
        {
            if (blockSize * numBlocks > array.Length)
            {
                Debug.LogError($"Invalid ReverseInBlocks, array length is {array.Length}, should be at least {blockSize * numBlocks}");
                return null;
            }

            if (s_ScratchSpace == null || s_ScratchSpace.Length < blockSize)
                s_ScratchSpace = new byte[blockSize];

            int startBlockIndex = 0;
            int endBlockIndex = ((int)numBlocks - 1) * blockSize;

            while (startBlockIndex < endBlockIndex)
            {
                Buffer.BlockCopy(array, startBlockIndex, s_ScratchSpace, 0, blockSize);
                Buffer.BlockCopy(array, endBlockIndex, array, startBlockIndex, blockSize);
                Buffer.BlockCopy(s_ScratchSpace, 0, array, endBlockIndex, blockSize);
                startBlockIndex += blockSize;
                endBlockIndex -= blockSize;
            }

            return array;
        }
}
