#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

class SlicedMapPublisher : public rclcpp::Node {
private:
  const double sliced_map_size = 50;
  int number_of_sliced_maps_x = 0;
  int number_of_sliced_maps_y = 0;
  std::string map_topic_name;
  std::string sliced_map_topic_name = "sliced_map";
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
  std::list<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> sliced_map_pubs;
  void map_callback(const nav_msgs::msg::OccupancyGrid msg) {
    number_of_sliced_maps_x = std::ceil(msg.info.width / sliced_map_size);
    number_of_sliced_maps_y = std::ceil(msg.info.height / sliced_map_size);
    int sliced_map_inc = number_of_sliced_maps_x * number_of_sliced_maps_y - sliced_map_pubs.size();
    if(sliced_map_inc > 0){
      for(int i = 0; i < sliced_map_inc; i++){
        std::string topic_name = sliced_map_topic_name + "/slice" + std::to_string(sliced_map_pubs.size() + 1);
        sliced_map_pubs.push_back(this->create_publisher<nav_msgs::msg::OccupancyGrid>(topic_name, 10));
      }
    }
    int initial_i = 0;
    int initial_j = 0;
    std::list<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr>::iterator it;
    for(it = sliced_map_pubs.begin(); it != sliced_map_pubs.end(); it++){
    auto sliced_map_msg = nav_msgs::msg::OccupancyGrid();
      sliced_map_msg.header = msg.header;
      sliced_map_msg.info = msg.info;
      sliced_map_msg.info.resolution = msg.info.resolution;
      sliced_map_msg.info.width = sliced_map_size;
      sliced_map_msg.info.height = sliced_map_size;
      sliced_map_msg.info.origin.position.x = msg.info.origin.position.x + initial_j * msg.info.resolution;
      sliced_map_msg.info.origin.position.y = msg.info.origin.position.y + initial_i * msg.info.resolution;
      sliced_map_msg.data.clear();
      fill_sliced_map(&msg.data, &sliced_map_msg.data, msg.info.height, msg.info.width, initial_i, initial_j);
      it->get()->publish(sliced_map_msg);
      initial_j += sliced_map_size;
      if(initial_j >= number_of_sliced_maps_x*sliced_map_size){
        initial_i += sliced_map_size;
        initial_j = 0;
      }
    }
  }
  void fill_sliced_map(const std::vector<int8_t> *data, std::vector<int8_t> *new_data, int height, int width, int initial_i, int initial_j){
    new_data->clear();
    for (int i = 0; i < sliced_map_size; i++) {
      for (int j = 0; j < sliced_map_size; j++) {
        // if there is not enough information to build the minimapl;
        bool i_exists = i + initial_i <= height;
        bool j_exists = j + initial_j <= width;
        bool ij_exists = (size_t)(i + initial_i) * width + j + initial_j <= data->size();
        if (i_exists && j_exists && ij_exists){
          new_data->push_back((*data)[(size_t)(i + initial_i) * width + j + initial_j]);
        }
        else
          new_data->push_back(-1);
      }
    }
  }

public:
  SlicedMapPublisher() : Node("MinimapPublisher_Node") {
  map_topic_name = this->declare_parameter<std::string>("map_topic_name", "map");
  map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic_name, 10, std::bind(&SlicedMapPublisher::map_callback, this, std::placeholders::_1));
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlicedMapPublisher>());
  rclcpp::shutdown();
}
