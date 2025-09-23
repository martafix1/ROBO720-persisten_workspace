#include <chrono>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class NoiseNode : public rclcpp::Node
{
public:
    NoiseNode() : Node("noise_cpp_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("noise", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&NoiseNode::publish_random_number, this));
        RCLCPP_INFO(this->get_logger(), "NoiseNode started.");
    }

private:
    void publish_random_number()
    {
        std_msgs::msg::Float64 msg;
        msg.data = distribution_(random_engine_);
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published: %f", msg.data);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::default_random_engine random_engine_;
    std::uniform_real_distribution<double> distribution_{0.0, 1.0};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NoiseNode>());
    rclcpp::shutdown();
    return 0;
}
