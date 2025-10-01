#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Node1 : public rclcpp::Node
{
public:
    Node1() : Node("node2")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic1", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                auto msg = std_msgs::msg::String();
                msg.data = "Hello from node1!";
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
                publisher_->publish(msg);
            });
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Node1>());
    rclcpp::shutdown();
    return 0;
}
