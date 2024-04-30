using System;
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Sensor;

// Code based on https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/7f055108f43ab11ff297175a52af5d30fbac9d57/com.unity.robotics.ros-tcp-connector/Runtime/MessageGeneration/MessageExtensions.cs
public class RosTcp_ImageSubscriber : MonoBehaviour{
    [Header("Image Settings")]
    [SerializeField] private string topicName = "image_raw";

    [Header("Image Dependencies")]
    [SerializeField] private RawImage display;

    [Header("Image Parameters")]
    [SerializeField] private bool debayer = false;
    [SerializeField] private bool convertBGR = true;
    [SerializeField] private bool flipY = true;

    private ROSConnection ros;
    private Texture2D texture = null;

    private void Start(){
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<ImageMsg>(topicName, ImageCallback);
    }

    private void ImageCallback(ImageMsg msg) {
        if(texture == null){
            if (debayer && msg.IsBayerEncoded()){
                texture = new Texture2D((int)msg.width / 2, (int)msg.height / 2, TextureFormat.RGBA32, false);
            }
            else{
                texture = new Texture2D((int)msg.width, (int)msg.height, msg.GetTextureFormat(), false);
            }
        }
        byte[] data;
        if (debayer && msg.IsBayerEncoded()){
            msg.DebayerConvert(flipY);
            data = msg.data;
        }
        else{
            data = EncodingConversion(msg, convertBGR, flipY);
        }

        texture.LoadRawTextureData(data);
        texture.Apply();
        display.texture = texture;      
    }

        static byte[] EncodingConversion(ImageMsg image, bool convertBGR = true, bool flipY = true)
        {
            // Number of channels in this encoding
            int channels = image.GetNumChannels();

            if (!image.EncodingRequiresBGRConversion())
                convertBGR = false;

            // If no modifications are necessary, return original array
            if (!convertBGR && !flipY)
                return image.data;

            int channelStride = image.GetBytesPerChannel();
            int pixelStride = channelStride * channels;
            int rowStride = pixelStride * (int)image.width;

            if (flipY)
            {
                ReverseInBlocks(image.data, rowStride, (int)image.height);
            }

            if (convertBGR)
            {
                // given two channels, we swap R with G (distance = 1).
                // given three or more channels, we swap R with B (distance = 2).
                int swapDistance = channels == 2 ? channelStride : channelStride * 2;
                int dataLength = (int)image.width * (int)image.height * pixelStride;

                if (channelStride == 1)
                {
                    // special case for the 1-byte-per-channel formats: avoid the inner loop
                    for (int pixelIndex = 0; pixelIndex < dataLength; pixelIndex += pixelStride)
                    {
                        int swapB = pixelIndex + swapDistance;
                        byte temp = image.data[pixelIndex];
                        image.data[pixelIndex] = image.data[swapB];
                        image.data[swapB] = temp;
                    }
                }
                else
                {
                    for (int pixelIndex = 0; pixelIndex < dataLength; pixelIndex += pixelStride)
                    {
                        int channelEndByte = pixelIndex + channelStride;
                        for (int byteIndex = pixelIndex; byteIndex < channelEndByte; byteIndex++)
                        {
                            int swapB = byteIndex + swapDistance;
                            byte temp = image.data[byteIndex];
                            image.data[byteIndex] = image.data[swapB];
                            image.data[swapB] = temp;
                        }
                    }
                }
            }
            return image.data;
        }

        static byte[] s_ScratchSpace;
        static void ReverseInBlocks(byte[] array, int blockSize, int numBlocks)
        {
            if (blockSize * numBlocks > array.Length)
            {
                Debug.LogError($"Invalid ReverseInBlocks, array length is {array.Length}, should be at least {blockSize * numBlocks}");
                return;
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
        }
}
