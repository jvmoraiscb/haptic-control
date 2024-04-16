using ROS2;
using System;
using UnityEngine;
using UnityEngine.UI;

// Code based on https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/7f055108f43ab11ff297175a52af5d30fbac9d57/com.unity.robotics.ros-tcp-connector/Runtime/MessageGeneration/MessageExtensions.cs
public class ImageSubscriber : MonoBehaviour{
    [Header("Image Settings")]
    [SerializeField] private string nodeName = "ImageSub_Unity";
    [SerializeField] private string topicName = "image_raw";

    [Header("Image Dependencies")]
    [SerializeField] private RawImage display;

    [Header("Image Parameters")]
    [SerializeField] private bool debayer = false;
    [SerializeField] private bool convertBGR = true;
    [SerializeField] private bool flipX = false;
    [SerializeField] private bool flipY = false;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<sensor_msgs.msg.Image> sub;
    private Texture2D texture = null;

    private sensor_msgs.msg.Image msg = null;

    private void Start(){
        ros2Unity = GetComponent<ROS2UnityComponent>();
    }

    private void Update(){
        if(!ros2Unity.Ok()) return;
        if(ros2Node == null){
            ros2Node = ros2Unity.CreateNode(nodeName);
            sub = ros2Node.CreateSubscription<sensor_msgs.msg.Image>(topicName, msg => ImageCallback(msg));
        }

        if(msg != null){
            if(texture == null){
                if (debayer && IsBayerEncoded(msg)){
                    texture = new Texture2D((int)msg.Width / 2, (int)msg.Height / 2, TextureFormat.RGBA32, false);
                }
                else{
                    texture = new Texture2D((int)msg.Width, (int)msg.Height, GetTextureFormat(msg), false);
                }
            }
            byte[] data;
            if (debayer && IsBayerEncoded(msg)){
                msg = DebayerConvert(msg, false);
                data = msg.Data;
            }
            else{
                data = EncodingConversion(msg, convertBGR, false);
            }

            texture.LoadRawTextureData(data);
            texture.Apply();
            display.texture = texture;
            var angleY = flipX ? 180 : 0;
            var angleX = flipY ? 180 : 0;
            display.transform.localRotation = Quaternion.Euler(angleX, angleY, 0);
        }
    }

    private void ImageCallback(sensor_msgs.msg.Image msg) {
        this.msg = msg;
    }

        static bool IsBayerEncoded(sensor_msgs.msg.Image image){
            switch (image.Encoding)
            {
                case "bayer_rggb8":
                case "bayer_bggr8":
                case "bayer_gbrg8":
                case "bayer_grbg8":
                case "bayer_rggb16":
                case "bayer_bggr16":
                case "bayer_gbrg16":
                case "bayer_grbg16":
                    return true;
                default:
                    return false;
            }
        }

        public static int GetBytesPerChannel(sensor_msgs.msg.Image image)
        {
            switch (image.Encoding)
            {
                case "8SC1":
                case "8SC2":
                case "8SC3":
                case "8SC4":
                case "8UC1":
                case "8UC2":
                case "8UC3":
                case "8UC4":
                case "mono8":
                case "bgr8":
                case "rgb8":
                case "bgra8":
                case "rgba8":
                case "bayer_rggb8":
                case "bayer_bggr8":
                case "bayer_gbrg8":
                case "bayer_grbg8":
                    return 1;
                case "16SC1":
                case "16SC2":
                case "16SC3":
                case "16SC4":
                case "16UC1":
                case "16UC2":
                case "16UC3":
                case "16UC4":
                case "mono16":
                case "bayer_rggb16":
                case "bayer_bggr16":
                case "bayer_gbrg16":
                case "bayer_grbg16":
                    return 2;
                case "32FC1":
                case "32SC1":
                case "32FC2":
                case "32SC2":
                case "32FC3":
                case "32SC3":
                case "32FC4":
                case "32SC4":
                    return 4;
                case "64FC1":
                case "64FC2":
                case "64FC3":
                case "64FC4":
                    return 8;
            }
            return 1;
        }

        static TextureFormat GetTextureFormat(sensor_msgs.msg.Image image)
        {
            switch (image.Encoding)
            {
                case "8UC1":
                case "8SC1":
                    return TextureFormat.R8;
                case "8UC2":
                case "8SC2":
                    return TextureFormat.RG16;
                case "8UC3":
                case "8SC3":
                    return TextureFormat.RGB24;
                case "8UC4":
                case "8SC4":
                case "bgra8":
                    return TextureFormat.BGRA32; // unity supports these natively
                case "16UC1":
                case "16SC1":
                    return TextureFormat.R16;
                case "16UC2":
                case "16SC2":
                    return TextureFormat.RG32;
                case "16UC3":
                case "16SC3":
                    return TextureFormat.RGB48;
                case "16UC4":
                case "16SC4":
                    return TextureFormat.RGBA64;
                case "32SC1":
                case "32SC2":
                case "32SC3":
                case "32SC4":
                    throw new NotImplementedException("32 bit integer texture formats are not supported");
                case "32FC1":
                    return TextureFormat.RFloat;
                case "32FC2":
                    return TextureFormat.RGFloat;
                case "32FC3":
                    throw new NotImplementedException("32FC3 texture format is not supported");
                case "32FC4":
                    return TextureFormat.RGBAFloat;
                case "64FC1":
                case "64FC2":
                case "64FC3":
                case "64FC4":
                    throw new NotImplementedException("Double precision texture formats are not supported");
                case "mono8":
                    return TextureFormat.R8;
                case "mono16":
                    return TextureFormat.R16;
                case "bgr8":
                    return TextureFormat.RGB24;
                case "rgb8":
                    return TextureFormat.RGB24; // unity supports this natively
                case "rgba8":
                    return TextureFormat.RGBA32; // unity supports this natively
                case "bayer_rggb8":
                case "bayer_bggr8":
                case "bayer_gbrg8":
                case "bayer_grbg8":
                    return TextureFormat.R8;
                case "bayer_rggb16":
                case "bayer_bggr16":
                case "bayer_gbrg16":
                case "bayer_grbg16":
                    return TextureFormat.R16;
            }
            return TextureFormat.RGB24;
        }

        public static int GetNumChannels(sensor_msgs.msg.Image image)
        {
            switch (image.Encoding)
            {
                case "8SC1":
                case "8UC1":
                case "16SC1":
                case "16UC1":
                case "32FC1":
                case "32SC1":
                case "64FC1":
                case "mono8":
                case "mono16":
                case "bayer_rggb8":
                case "bayer_bggr8":
                case "bayer_gbrg8":
                case "bayer_grbg8":
                case "bayer_rggb16":
                case "bayer_bggr16":
                case "bayer_gbrg16":
                case "bayer_grbg16":
                    return 1;
                case "8SC2":
                case "8UC2":
                case "16SC2":
                case "16UC2":
                case "32FC2":
                case "32SC2":
                case "64FC2":
                    return 2;
                case "8SC3":
                case "8UC3":
                case "16SC3":
                case "16UC3":
                case "32FC3":
                case "32SC3":
                case "64FC3":
                case "bgr8":
                case "rgb8":
                    return 3;
                case "8SC4":
                case "8UC4":
                case "16SC4":
                case "16UC4":
                case "32FC4":
                case "32SC4":
                case "64FC4":
                case "bgra8":
                case "rgba8":
                    return 4;
            }
            return 4;
        }

        public static bool EncodingRequiresBGRConversion(sensor_msgs.msg.Image image)
        {
            switch (image.Encoding)
            {
                case "8SC1":
                case "8UC1":
                case "16SC1":
                case "16UC1":
                case "32FC1":
                case "32SC1":
                case "64FC1":
                case "mono8":
                case "mono16":
                    // single channel = nothing to swap
                    return false;
                case "8UC4":
                case "8SC4":
                case "bgra8":
                    return false; // raw BGRA32 texture format
                case "rgb8":
                    return false; // raw RGB24 texture format
                case "rgba8":
                    return false; // raw RGB32 texture format
                case "bayer_rggb8":
                case "bayer_bggr8":
                case "bayer_gbrg8":
                case "bayer_grbg8":
                    return false; // bayer has its own conversions needed
                default:
                    return true;
            }
        }

        static byte[] EncodingConversion(sensor_msgs.msg.Image image, bool convertBGR = true, bool flipY = true){
            // Number of channels in this encoding
            int channels = GetNumChannels(image);

            if (!EncodingRequiresBGRConversion(image))
                convertBGR = false;

            // If no modifications are necessary, return original array
            if (!convertBGR && !flipY)
                return image.Data;

            int channelStride = GetBytesPerChannel(image);
            int pixelStride = channelStride * channels;
            int rowStride = pixelStride * (int)image.Width;

            if (flipY){
                image.Data = ReverseInBlocks(image.Data, rowStride, (int)image.Height);
            }

            if (convertBGR){
                // given two channels, we swap R with G (distance = 1).
                // given three or more channels, we swap R with B (distance = 2).
                int swapDistance = channels == 2 ? channelStride : channelStride * 2;
                int dataLength = (int)image.Width * (int)image.Height * pixelStride;

                if (channelStride == 1){
                    // special case for the 1-byte-per-channel formats: avoid the inner loop
                    for (int pixelIndex = 0; pixelIndex < dataLength; pixelIndex += pixelStride)
                    {
                        int swapB = pixelIndex + swapDistance;
                        byte temp = image.Data[pixelIndex];
                        image.Data[pixelIndex] = image.Data[swapB];
                        image.Data[swapB] = temp;
                    }
                }
                else{
                    for (int pixelIndex = 0; pixelIndex < dataLength; pixelIndex += pixelStride)
                    {
                        int channelEndByte = pixelIndex + channelStride;
                        for (int byteIndex = pixelIndex; byteIndex < channelEndByte; byteIndex++)
                        {
                            int swapB = byteIndex + swapDistance;
                            byte temp = image.Data[byteIndex];
                            image.Data[byteIndex] = image.Data[swapB];
                            image.Data[swapB] = temp;
                        }
                    }
                }
            }
            return image.Data;
        }

        static byte[] ReverseInBlocks(byte[] array, int blockSize, int numBlocks)
        {
            if (blockSize * numBlocks > array.Length)
            {
                Debug.LogError($"Invalid ReverseInBlocks, array length is {array.Length}, should be at least {blockSize * numBlocks}");
                return null;
            }
            byte[] s_ScratchSpace = new byte[blockSize];

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

        static byte[] s_ScratchSpaceD;
        static sensor_msgs.msg.Image DebayerConvert(sensor_msgs.msg.Image image, bool flipY = true)
        {
            int channelStride = GetBytesPerChannel(image);
            int width = (int)image.Width;
            int height = (int)image.Height;
            int rowStride = width * channelStride;
            int dataSize = rowStride * height;
            int finalPixelStride = channelStride * 4;

            int[] reorderIndices;
            switch (image.Encoding)
            {
                case "bayer_rggb8":
                    reorderIndices = new int[] { 0, 1, width + 1 };
                    break;
                case "bayer_bggr8":
                    reorderIndices = new int[] { width + 1, 1, 0 };
                    break;
                case "bayer_gbrg8":
                    reorderIndices = new int[] { width, 0, 1 };
                    break;
                case "bayer_grbg8":
                    reorderIndices = new int[] { 1, 0, width };
                    break;
                case "bayer_rggb16":
                    reorderIndices = new int[] { 0, 1, 2, 3, rowStride + 2, rowStride + 3 };
                    break;
                case "bayer_bggr16":
                    reorderIndices = new int[] { rowStride + 2, rowStride + 3, 2, 3, 0, 1 };
                    break;
                case "bayer_gbrg16":
                    reorderIndices = new int[] { rowStride, rowStride + 1, 0, 1, 2, 3 };
                    break;
                case "bayer_grbg16":
                    reorderIndices = new int[] { 2, 3, 0, 1, rowStride, rowStride + 1 };
                    break;
                default:
                    return null;
            }

            if (flipY){
                image.Data = ReverseInBlocks(image.Data, rowStride * 2, (int)image.Height / 2);
            }

            if (s_ScratchSpaceD == null || s_ScratchSpaceD.Length < rowStride * 2)
                s_ScratchSpaceD = new byte[rowStride * 2];

            int rowStartIndex = 0;
            while (rowStartIndex < dataSize)
            {
                Buffer.BlockCopy(image.Data, rowStartIndex, s_ScratchSpaceD, 0, rowStride * 2);
                int pixelReadIndex = 0;
                int pixelWriteIndex = rowStartIndex;
                while (pixelReadIndex < rowStride)
                {
                    for (int Idx = 0; Idx < reorderIndices.Length; ++Idx)
                    {
                        image.Data[pixelWriteIndex + Idx] = s_ScratchSpaceD[pixelReadIndex + reorderIndices[Idx]];
                    }
                    image.Data[pixelWriteIndex + reorderIndices.Length] = 255;
                    if (channelStride == 2)
                        image.Data[pixelWriteIndex + reorderIndices.Length + 1] = 255;
                    pixelReadIndex += channelStride * 2;
                    pixelWriteIndex += finalPixelStride;
                }
                rowStartIndex += rowStride * 2;
            }

            image.Width = image.Width / 2;
            image.Height = image.Height / 2;
            image.Encoding = channelStride == 1 ? "rgba8" : "rgba16";
            image.Step = (uint)(channelStride * image.Width);

            return image;
        }
}
