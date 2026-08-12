#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>

class object_tracking_aruco : public rclcpp::Node{
    public:
    object_tracking_aruco():Node("objecttracking_aruco_node_cpp"){
        RCLCPP_INFO(this->get_logger(), "Omnibot Node CPP Started!");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<object_tracking_aruco>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}