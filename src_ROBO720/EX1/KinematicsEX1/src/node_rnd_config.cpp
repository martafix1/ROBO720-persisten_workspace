#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <random>

class RandomNumberNode : public rclcpp::Node
{
public:
  RandomNumberNode() : Node("rnd_configurable"), gen_(rd_())
  {
    // Declare parameters with default values
    this->declare_parameter<double>("min", 0.0);
    this->declare_parameter<double>("max", 1.0);
    this->declare_parameter<int>("period_ms", 1000);

    // Initialize parameters
    min_ = this->get_parameter("min").as_double();
    max_ = this->get_parameter("max").as_double();
    period_ms_ = this->get_parameter("period_ms").as_int();

    publisher_ = this->create_publisher<std_msgs::msg::Float64>("random_number", 10);

    // Setup timer with initial period
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms_),
      std::bind(&RandomNumberNode::timer_callback, this));

    // Register callback to handle parameter changes dynamically
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&RandomNumberNode::on_params_changed, this, std::placeholders::_1));
  }

private:
  void timer_callback()
  {
    if (max_ < min_) {
      RCLCPP_WARN(this->get_logger(), "max < min, swapping values");
      std::swap(min_, max_);
    }

    std::uniform_real_distribution<double> dist(min_, max_);
    double rnd_val = dist(gen_);

    auto msg = std_msgs::msg::Float64();
    msg.data = rnd_val;
    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Published random number: %.4f", rnd_val);
  }

  rcl_interfaces::msg::SetParametersResult on_params_changed(const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    bool need_restart_timer = false;

    for (const auto & param : params) {
      if (param.get_name() == "min") {
        min_ = param.as_double();
      } else if (param.get_name() == "max") {
        max_ = param.as_double();
      } else if (param.get_name() == "period_ms") {
        int new_period = param.as_int();
        if (new_period != period_ms_ && new_period > 0) {
          period_ms_ = new_period;
          need_restart_timer = true;
        } else if (new_period <= 0) {
          result.successful = false;
          result.reason = "period_ms must be > 0";
        }
      }
    }

    if (need_restart_timer) {
      timer_->cancel();
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms_),
        std::bind(&RandomNumberNode::timer_callback, this));
      RCLCPP_INFO(this->get_logger(), "Timer period updated to %d ms", period_ms_);
    }

    return result;
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::random_device rd_;
  std::mt19937 gen_;

  double min_;
  double max_;
  int period_ms_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RandomNumberNode>());
  rclcpp::shutdown();
  return 0;
}