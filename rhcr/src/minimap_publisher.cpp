#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

/*
  Using Minimap on launch.py:
    minimap_node = launch_ros.actions.Node(
      package=package_name,
      executable='minimap_publisher',
      name='minimap_publisher_launch',
      parameters=[
          {'odom_topic_name': 'unity_odom'},
          {'map_topic_name': 'map'},
          {'minimap_topic_name': 'minimap'}
      ]
    )
*/
class MinimapPublisher : public rclcpp::Node {
private:
  std::string odom_topic_name;
  std::string map_topic_name;
  std::string minimap_topic_name;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  double robot_x;
  double robot_y;
  const int minimap_side = 11;
  void map_callback(const nav_msgs::msg::OccupancyGrid msg) {
    // the origin of the map is indicative of the cell (0,0). By subtracting the robot's position from the origin, the distance from the robot to the origin is obtained. Dividing this value by the map resolution allows for the identification of the cell representing that distance.
    int robot_x_on_map = std::round((robot_x - msg.info.origin.position.x) / msg.info.resolution);
    int robot_y_on_map = std::round((robot_y - msg.info.origin.position.y) / msg.info.resolution);

    // with the minimap being 101 by 101, after identifying the x and y coordinates of the robot on the map, the designated position is set as the center of the map (map[50][50]).
    int initial_i = robot_y_on_map - (minimap_side - 1) / 2;
    int initial_j = robot_x_on_map - (minimap_side - 1) / 2;

    auto minimap = nav_msgs::msg::OccupancyGrid();
    minimap.header = msg.header;
    minimap.info = msg.info;
    minimap.info.resolution = msg.info.resolution;
    minimap.info.width = minimap_side;
    minimap.info.height = minimap_side;
    minimap.info.origin.position.x = msg.info.origin.position.x + initial_j * msg.info.resolution;
    minimap.info.origin.position.y = msg.info.origin.position.y + initial_i * msg.info.resolution;
    minimap.data.clear();
    for (int i = 0; i < minimap_side; i++) {
      for (int j = 0; j < minimap_side; j++) {
        // if there is not enough information to build the minimap
        bool i_exists = (size_t)(i + initial_i) < msg.info.height;
        bool j_exists = (size_t)(j + initial_j) < msg.info.width;
        bool ij_exists = (size_t)(i + initial_i) * msg.info.width + j + initial_j < msg.data.size();
        if (i_exists && j_exists && ij_exists)
          minimap.data.push_back(msg.data[(i + initial_i) * msg.info.width + j + initial_j]);
        else
          minimap.data.push_back(-1);
      }
    }
    // the minimap is only published when received, obviously
    map_pub->publish(minimap);
  }
  void odom_callback(const nav_msgs::msg::Odometry msg) {
    robot_x = msg.pose.pose.position.x;
    robot_y = msg.pose.pose.position.y;
  }

public:
  MinimapPublisher() : Node("MinimapPublisher_Node") {
    odom_topic_name = this->declare_parameter<std::string>("odom_topic_name", "odom");
    map_topic_name = this->declare_parameter<std::string>("map_topic_name", "map");
    minimap_topic_name = this->declare_parameter<std::string>("minimap_topic_name", "minimap");
    map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(minimap_topic_name, 10);
    map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic_name, 10, std::bind(&MinimapPublisher::map_callback, this, std::placeholders::_1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_name, 10, std::bind(&MinimapPublisher::odom_callback, this, std::placeholders::_1));
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimapPublisher>());
  rclcpp::shutdown();
}
