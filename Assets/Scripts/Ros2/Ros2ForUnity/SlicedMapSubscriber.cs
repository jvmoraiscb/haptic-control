using UnityEngine;
using ROS2;
using System.Collections.Generic;

public class SlicedMapSubscriber : MonoBehaviour{
    [Header("Sliced Map Settings")]
    [SerializeField] private string nodeName = "SlicedMapSub_Unity";
    [SerializeField] private string topicName = "sliced_map/slice";

    [Header("Sliced Map Dependencies")]
    [SerializeField] private GameObject wallPrefab;
    [SerializeField] private int numberOfSlices = 0;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private List<List<GameObject>> listOfSlicedMaps;
    private List<nav_msgs.msg.OccupancyGrid> listOfMsgs;

    private void Start(){
        ros2Unity = GetComponent<ROS2UnityComponent>();
        listOfSlicedMaps = new List<List<GameObject>>();
        listOfMsgs = new List<nav_msgs.msg.OccupancyGrid>();
    }

    private void Update(){
        if(!ros2Unity.Ok()) return;
        if(ros2Node == null){
            ros2Node = ros2Unity.CreateNode(nodeName);
            for(int i = 1; i <= numberOfSlices; i++){
                listOfMsgs.Add(null);
                listOfSlicedMaps.Add(new List<GameObject>());
                ros2Node.CreateSubscription<nav_msgs.msg.OccupancyGrid>(topicName + i.ToString(), msg => SlicedMapCallback(msg, i));
            }
        }
        int slice = 0;
        foreach(nav_msgs.msg.OccupancyGrid msg in listOfMsgs){
            if(msg != null){
                foreach (GameObject obj in listOfSlicedMaps[slice]){
                    Destroy(obj);
                }
                listOfSlicedMaps[slice].Clear();
                // convert to unity system
                var origin = Transformations.Ros2Unity(new Vector3{
                    x = (float)msg.Info.Origin.Position.X,
                    y = (float)msg.Info.Origin.Position.Y,
                    z = (float)msg.Info.Origin.Position.Z
                });
                for(int i = 0; i < msg.Info.Height; i++){
                    for(int j = 0; j < msg.Info.Width; j++){
                        if(msg.Data[i*msg.Info.Width + j] == 100){
                            var newPose = Transformations.Ros2Unity(new Vector3{
                                x = j*msg.Info.Resolution,
                                y = i*msg.Info.Resolution,
                                z = 0f
                            });

                            var point = new Vector3(origin.x + newPose.x, 0f, origin.z + newPose.z);
                            listOfSlicedMaps[slice].Add(Instantiate(wallPrefab, point, new Quaternion(0, 0, 0, 1)));
                        }
                    }
                }
                slice++;
            }
        }
    }

    private void SlicedMapCallback(nav_msgs.msg.OccupancyGrid msg, int slice){
        listOfMsgs[slice] = msg;
    }
}