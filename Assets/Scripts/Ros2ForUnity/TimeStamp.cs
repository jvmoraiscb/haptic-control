using System;
using UnityEngine;

namespace ROS2{
    // Code based on https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/main/Nav2SLAMExampleProject/Assets/Scripts/TimeStamp.cs
    public readonly struct TimeStamp{
        public const double k_NanosecondsInSecond = 1e9f;

        // TODO: specify base time this stamp is measured against (Sim 0, time since application start, etc.)
        public readonly int Seconds;
        public readonly uint NanoSeconds;

        // (From Unity Time.time)
        public TimeStamp(double timeInSeconds){
            var sec = Math.Floor(timeInSeconds);
            var nsec = (timeInSeconds - sec) * k_NanosecondsInSecond;
            // TODO: Check for negatives to ensure safe cast
            Seconds = (int)sec;
            NanoSeconds = (uint)nsec;
        }

        // (From a ROS2 Time message)
        TimeStamp(int sec, uint nsec){
            Seconds = sec;
            NanoSeconds = nsec;
        }

        // NOTE: We could define these operators in a transport-specific extension package
        public static implicit operator builtin_interfaces.msg.Time(TimeStamp stamp){
            return new builtin_interfaces.msg.Time{
                Sec = stamp.Seconds,
                Nanosec = stamp.NanoSeconds
                };
        }

        public static implicit operator TimeStamp(builtin_interfaces.msg.Time stamp){
            return new TimeStamp(stamp.Sec, stamp.Nanosec);
        }
    }
}