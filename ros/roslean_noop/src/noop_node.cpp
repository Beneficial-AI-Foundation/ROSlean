// No-op ROS 2 node backed by a Lean-compiled function.
//
// Once per second it passes a sequence number through `ROSleanCore.noop`
// (exported from Lean as `roslean_noop`) and publishes the result on
// `~/heartbeat`. It does nothing useful; it proves that Lean code can be
// initialised and called from inside an rclcpp node.

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "roslean_core/mux.hpp"
#include "std_msgs/msg/u_int64.hpp"

namespace
{

class NoopNode : public rclcpp::Node
{
public:
  NoopNode()
  : Node("roslean_noop")
  {
    pub_ = create_publisher<std_msgs::msg::UInt64>("~/heartbeat", 10);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] {tick();});
    RCLCPP_INFO(get_logger(), "Lean core initialised; publishing on ~/heartbeat");
  }

private:
  void tick()
  {
    std_msgs::msg::UInt64 msg;
    msg.data = roslean_noop(seq_);
    if (msg.data != seq_) {
      RCLCPP_ERROR(
        get_logger(), "Lean noop returned %llu for %llu",
        static_cast<unsigned long long>(msg.data), static_cast<unsigned long long>(seq_));
    }
    RCLCPP_INFO(get_logger(), "lean noop(%llu) = %llu",
      static_cast<unsigned long long>(seq_), static_cast<unsigned long long>(msg.data));
    pub_->publish(msg);
    ++seq_;
  }

  uint64_t seq_{0};
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  roslean::initialize();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NoopNode>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
