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

#include <lean/lean.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

extern "C" {
// Lean runtime and module initialisers (see the generated .lake/build/ir/*.c).
void lean_initialize_runtime_module();
lean_object * initialize_ROSlean_ROSleanCore(uint8_t builtin);
// @[export roslean_noop] def ROSleanCore.noop (seq : UInt64) : UInt64
uint64_t roslean_noop(uint64_t seq);
}

namespace
{

// Must run once, before any Lean function is called.
bool init_lean()
{
  lean_initialize_runtime_module();
  lean_object * res = initialize_ROSlean_ROSleanCore(1 /* builtin */);
  if (!lean_io_result_is_ok(res)) {
    lean_io_result_show_error(res);
    lean_dec(res);
    return false;
  }
  lean_dec_ref(res);
  lean_io_mark_end_initialization();
  return true;
}

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
  if (!init_lean()) {
    return EXIT_FAILURE;
  }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NoopNode>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
