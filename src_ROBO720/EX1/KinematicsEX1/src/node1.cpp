#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <iostream>
#include <Eigen/Geometry>


#include <random>

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");


class Node1 : public rclcpp::Node
{
public:
    Node1() : Node("node1")
    {
        // Set up publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic1", 10);

        // Timer runs every 10 seconds
        timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&Node1::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Node1 construction finnished");
    }

private:
    // Generate random doubles between min and max
    double generate_random(double min, double max)
    {
        std::uniform_real_distribution<double> dist(min, max);
        return dist(rng_);
    }

    // Called every timer tick
    void timer_callback()
    {
        // Generate two random numbers
        double input1 = generate_random(-1.0, 1.0);
        double input2 = generate_random(-1.0, 1.0);

        // Perform kinematics using them (you implement this)
        std::string result = perform_kinematics(input1, input2);

        // Publish result
        auto msg = std_msgs::msg::String();
        msg.data = result;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
        publisher_->publish(msg);
    }

    // Your placeholder for kinematics logic
    std::string perform_kinematics(double input1, double input2)
    {
        // TODO: use KDL, Eigen, etc., to compute something
        
        std::ostringstream oss;
        oss << "Kinematics result with input1 = " << input1 << ", input2 = " << input2;

        // Example: return formatted string
        return oss.str();
    }

    // Members
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mt19937 rng_{std::random_device{}()};  // random number generator
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Node1>());
    rclcpp::shutdown();
    return 0;
}
