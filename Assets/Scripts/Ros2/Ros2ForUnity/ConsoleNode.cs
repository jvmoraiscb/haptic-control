using System;
using UnityEngine;
using ROS2;
using System.Linq;

public class ConsoleNode : MonoBehaviour{
    [Header("Console Settings")]
    [SerializeField] private string nodeName = "ConsoleNode_Unity";
    [SerializeField] private string consoleTopicName = "console";

    [Header("Console Dependencies")]
    [SerializeField] private GameObject wallPrefab = null;

    // ros2 variables
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;

    // virtual map variables
    private System.Collections.Generic.List<GameObject> walls;
    private Vector3 position;
    private Quaternion rotation;
    private Vector3 scale;
    private bool shouldAddWall = false;
    private bool shouldDeleteWall = false;
    private bool shouldDeleteAllWalls = false;

    void Start(){
        walls = new System.Collections.Generic.List<GameObject>();
        ros2Unity = GetComponent<ROS2UnityComponent>();
        ros2Node ??= ros2Unity.CreateNode(nodeName);
        if (ros2Unity.Ok()){
            ros2Node.CreateSubscription<std_msgs.msg.String>(consoleTopicName, msg => ConsoleHandler(msg));
        }
    }

    void Update(){
        if(shouldAddWall){
            GameObject newWall = Instantiate(wallPrefab, position, rotation);
            newWall.transform.localScale = scale;
            walls.Add(newWall);
            shouldAddWall = false;
        }
        if(shouldDeleteWall){
            foreach (GameObject wall in walls.ToList()){
                bool isInXAxis = wall.transform.localPosition.x - wall.transform.localScale.x < position.x && wall.transform.localPosition.x + wall.transform.localScale.x > position.x;
                bool isInZAxis = wall.transform.localPosition.z - wall.transform.localScale.z < position.z && wall.transform.localPosition.z + wall.transform.localScale.z > position.z;
                if(isInXAxis && isInZAxis){
                    walls.Remove(wall);
                    Destroy(wall);
                }
            }
            shouldDeleteWall = false;
        }
        if(shouldDeleteAllWalls){
            foreach (GameObject wall in walls.ToList()){
                Destroy(wall);
            }
            walls.Clear();
            shouldDeleteAllWalls = false;
        }
    }

    void ConsoleHandler(std_msgs.msg.String msg){
        string[] args = msg.Data.Split(' ');
        if (args.Length > 1){
            if(args[0] == "virtual-map")
                VirtualMapOption(args);
        }
    }

    // virtual-map remove pos-x pos-y pos-z
    void VirtualMapOption(string[] args){
        if(args.Length >= 2){
            // virtual-map create pos-x pos-y pos-z quat-x quat-y quat-z quat-w x-scale y-scale z-scale
            if(args[1] == "create" && args.Length == 12){
                float posX, posY, posZ, quatX, quatY, quatZ, quatW, scaleX, scaleY, scaleZ;
                try{
                    IFormatProvider format = System.Globalization.CultureInfo.InvariantCulture.NumberFormat;
                    posX = float.Parse(args[2], format);
                    posY = float.Parse(args[3], format);
                    posZ = float.Parse(args[4], format);
                    quatX = float.Parse(args[5], format);
                    quatY = float.Parse(args[6], format);
                    quatZ = float.Parse(args[7], format);
                    quatW = float.Parse(args[8], format);
                    scaleX = float.Parse(args[9], format);
                    scaleY = float.Parse(args[10], format);
                    scaleZ = float.Parse(args[11], format);
                }
                catch{
                    return;
                }
                position = Transformations.Ros2Unity(new Vector3(posX, posY, posZ));
                rotation = Transformations.Ros2Unity(new Quaternion(quatX, quatY, quatZ, quatW));
                scale = Transformations.Ros2Unity(new Vector3(scaleX, scaleY, scaleZ));
                scale.x = Math.Abs(scale.x);
                scale.y = Math.Abs(scale.y);
                scale.z = Math.Abs(scale.z);
                shouldAddWall = true;
            }
            // virtual-map delete pos-x pos-y pos-z
            if(args[1] == "delete" && args.Length == 5){
                float posX, posY, posZ;
                try{
                    IFormatProvider format = System.Globalization.CultureInfo.InvariantCulture.NumberFormat;
                    posX = float.Parse(args[2], format);
                    posY = float.Parse(args[3], format);
                    posZ = float.Parse(args[4], format);
                }
                catch{
                    return;
                }
                position = Transformations.Ros2Unity(new Vector3(posX, posY, posZ));
                shouldDeleteWall = true;
            }
            // virtual-map delete-all
            if(args[1] == "delete-all" && args.Length == 2){
                shouldDeleteAllWalls = true;
            }
        }
    }
}