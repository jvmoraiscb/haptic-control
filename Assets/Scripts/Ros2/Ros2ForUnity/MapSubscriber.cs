using UnityEngine;
using ROS2;
using System.Collections.Generic;

public class MapSubscriber : MonoBehaviour{
    [Header("Map Settings")]
    [SerializeField] private string nodeName = "MapSub_Unity";
    [SerializeField] private string topicName = "map";

    [Header("Map Dependencies")]
    [SerializeField] private GameObject pointPrefab;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private List<GameObject> listOfPointsObjects;
    private nav_msgs.msg.OccupancyGrid msg = null;

    private void Start(){
        ros2Unity = GetComponent<ROS2UnityComponent>();
        listOfPointsObjects = new List<GameObject>();
    }

    private void Update(){
        if(!ros2Unity.Ok()) return;
        if(ros2Node == null){
            ros2Node = ros2Unity.CreateNode(nodeName);
            ros2Node.CreateSubscription<nav_msgs.msg.OccupancyGrid>(topicName, msg => MapCallback(msg));
        }

        if(msg != null){
                foreach (GameObject obj in listOfPointsObjects){
                    Destroy(obj);
                }
                listOfPointsObjects.Clear();
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
                            listOfPointsObjects.Add(Instantiate(pointPrefab, point, new Quaternion(0, 0, 0, 1)));
                        }
                    }
                }
            }
    }

    private void MapCallback(nav_msgs.msg.OccupancyGrid msg){
        this.msg = msg;
    }
}