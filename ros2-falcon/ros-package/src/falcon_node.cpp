#include "falcon_node.hpp"

using namespace std;
using namespace chrono_literals;

Falcon_Node::Falcon_Node(Falcon* falcon, bool* debug_mode) {
    falcon_ = falcon;
    debug_mode_ = debug_mode;
    timer_ = this->create_wall_timer(10ms, std::bind(&Falcon_Node::timer_callback, this));

    position_vector_pub = this->create_publisher<geometry_msgs::msg::Vector3>("position_vector", 10);
    right_button_pub = this->create_publisher<std_msgs::msg::Int16>("right_button", 10);
    up_button_pub = this->create_publisher<std_msgs::msg::Int16>("up_button", 10);
    center_button_pub = this->create_publisher<std_msgs::msg::Int16>("center_button", 10);
    left_button_pub = this->create_publisher<std_msgs::msg::Int16>("left_button", 10);
    force_vector_sub = this->create_subscription<geometry_msgs::msg::Vector3>("force_vector", 10, std::bind(&Falcon_Node::force_callback, this, std::placeholders::_1));
    rgb_vector_sub = this->create_subscription<geometry_msgs::msg::Vector3>("rgb_vector", 10, std::bind(&Falcon_Node::rgb_callback, this, std::placeholders::_1));

    printf("Please calibrate the controller: move it around and then press the center button.\n");

    falcon_->rgb(true, false, false);
    falcon_->calibrate();
    falcon_->rgb(false, true, false);

    falcon_->print_info();
}

void Falcon_Node::timer_callback() {
    double x, y, z;
    int button1, button2, button3, button4;

    falcon_->update();
    falcon_->get(&x, &y, &z, &button1, &button2, &button3, &button4);

    if (*debug_mode_) {
        printf("X: %+.2f | Y: %+.2f | Z: %+.2f | B1: %d | B2: %d | B3: %d | B4: %d\n", x, y, z, button1, button2, button3, button4);
    }

    auto pos = geometry_msgs::msg::Vector3();
    pos.x = (float)x;
    pos.y = (float)y;
    pos.z = (float)z;

    auto right = std_msgs::msg::Int16();
    right.data = button1;

    auto up = std_msgs::msg::Int16();
    up.data = button2;

    auto center = std_msgs::msg::Int16();
    center.data = button3;

    auto left = std_msgs::msg::Int16();
    left.data = button4;

    position_vector_pub->publish(pos);
    right_button_pub->publish(right);
    up_button_pub->publish(up);
    center_button_pub->publish(center);
    left_button_pub->publish(left);
}

void Falcon_Node::force_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    falcon_->set(msg->x, msg->y, msg->z);
}

void Falcon_Node::rgb_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    bool red = ((int)msg->x == 1) ? true : false;
    bool green = ((int)msg->y == 1) ? true : false;
    bool blue = ((int)msg->z == 1) ? true : false;

    falcon_->rgb(red, green, blue);
}