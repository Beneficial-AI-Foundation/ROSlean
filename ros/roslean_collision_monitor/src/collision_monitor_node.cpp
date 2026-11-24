// Collision monitor with the decision logic in Lean: a lifecycle node that
// Nav2 can manage in place of nav2_collision_monitor.
//
// This file is the shim: it reads parameters, creates subscriptions, reads the
// clock, looks transforms up and publishes. It contains no decision logic and
// converts no configuration value: parameters go to the Lean core as they
// were read (doubles and floats as bit patterns), and the core converts them,
// refuses what the node does not accept, and says which parameter it refused.
//
// What the core is handed (docs/collision_monitor/SPEC_NAV2.md, section 8):
//   - every lifecycle transition, from its `on_...` callback;
//   - a scan as the bit patterns of its ranges;
//   - a polygon message for a zone fed by a topic as its vertices in the
//     base frame;
//   - a command as six bit patterns, together with the points of the stored
//     scan in the base frame, one pair per reading, under the transform
//     looked up for this command, and the vertices of the stored polygon
//     messages that are in another frame.
// The core answers a command with: nothing at all; or a velocity (the
// command, zero, the command scaled by a ratio) or the notice that the
// velocity is withheld, together with the state message to publish, if any.
// The shim performs no floating-point test on a command and keeps no flag
// "active" of its own: that nothing is published while the node is not
// active is the core's answer.
//
// What the shim computes is trusted and sits in two headers with their own
// tests: transforms.hpp (the points of a scan and the vertices of a polygon
// message in the base frame: T2, T6) and conversions.hpp (the slowdown
// multiplication, the `points` string, what is published for an answer: T1,
// T3, T5, T9).
//
// Parameters are read in `on_configure`, as upstream reads them. A change
// through the parameter service is rejected (ND8), and a name that was not
// given at start-up cannot be set at all.
//
// Rule (R..., N...) and difference (D..., ND...) numbers refer to SPEC.md and
// SPEC_NAV2.md in docs/collision_monitor.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_msgs/msg/collision_monitor_state.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "roslean_collision_monitor/conversions.hpp"
#include "roslean_collision_monitor/transforms.hpp"
#include "roslean_core/collision_monitor.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

namespace roslean_collision_monitor
{

using roslean::CollisionMonitor;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// What was read for a zone, and what the shim keeps for it: the lookups of
/// a zone fed by a topic (T6) and what is displayed (T8). The core has its
/// own copy of everything that enters a decision, and decides with that.
struct ZoneParams
{
  std::string name;
  bool circle{false};
  bool slowdown{false};
  bool enabled{true};
  double radius{0.0};
  double ratio{0.0};
  int64_t min_points{0};
  std::vector<std::array<float, 2>> vertices;
  /// The shape arrives on `sub_topic` (N31).
  bool topic{false};
  std::string sub_topic;
  bool transient_local{false};
  /// N16.
  bool visualize{false};
  std::string pub_topic;
  /// The polygon message the core accepted last for this zone (N33): its
  /// frame and points are what the lookups of later commands use (N34).
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr shape;
  /// What is published for the zone (N16).
  geometry_msgs::msg::PolygonStamped display;
  rclcpp::SubscriptionBase::SharedPtr sub;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub;
};

/// Twice the signed area of a polygon whose vertices are whole millimetres
/// below 2^20 in magnitude: exact in 64-bit integers. For a warning only.
inline int64_t twice_area_mm2(const std::vector<std::array<int64_t, 2>> & v)
{
  int64_t sum = 0;
  for (size_t i = 0; i < v.size(); ++i) {
    const auto & a = v[i];
    const auto & b = v[(i + 1) % v.size()];
    sum += a[0] * b[1] - b[0] * a[1];
  }
  return sum;
}

inline rclcpp::NodeOptions node_options(rclcpp::NodeOptions options)
{
  // Every parameter given at start-up is declared; no other name can be set
  // later (finding 2 of the review CM3/CM4).
  return options.allow_undeclared_parameters(false)
         .automatically_declare_parameters_from_overrides(true)
         // D15: rclcpp takes /clock messages in a thread of its own by
         // default, and a message with a negative time makes it throw there,
         // where nothing can catch it: the process ends. Without that thread
         // the message is taken in by the executor that spins this node, and
         // the exception arrives in main(), which goes on (see there).
         .use_clock_thread(false);
}

class CollisionMonitorNode : public nav2_util::LifecycleNode
{
public:
  explicit CollisionMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : nav2_util::LifecycleNode("collision_monitor", "", node_options(options))
  {
    roslean::initialize();
    // ND8: parameters are read in on_configure. A change through the
    // parameter service is rejected, with the reason, instead of being
    // accepted and ignored.
    on_set_parameters_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & changes) {
        rcl_interfaces::msg::SetParametersResult result;
        if (configuring_) {
          // Declarations made by what on_configure creates (the QoS
          // overrides of the TF listener's subscriptions) are not changes.
          result.successful = true;
          return result;
        }
        result.successful = false;
        result.reason = "parameters of this node are read once, when it is configured, from "
        "the values given at start-up, and cannot be changed while it runs "
        "(docs/collision_monitor/SPEC_NAV2.md, ND8)";
        if (!changes.empty()) {
          result.reason += "; to change '" + changes.front().get_name() +
          "', restart the node with the new value";
        }
        return result;
      });
  }

  /// An exception left a callback and was caught in main(). Under simulated
  /// time the one known cause is a /clock message that rclcpp refuses (D15),
  /// so the clock counts as not valid until a usable message arrives; the
  /// node's own look at the message says the same, and whichever of the two
  /// comes first counts.
  void on_callback_exception(const char * what)
  {
    RCLCPP_ERROR(
      get_logger(), "An exception left a callback and was caught; the node goes on: %s", what);
    if (sim_time_) {
      set_clock_valid(false);
    }
  }

protected:
  // ---- lifecycle (N2 to N8) ----------------------------------------------

  /// N3. Reads the parameters, builds the core and, if the core accepts the
  /// configuration, creates the TF buffer, the subscriptions and the
  /// publishers. A refused configuration fails the transition: the node
  /// stays unconfigured and the process stays up, as upstream.
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    configuring_ = true;
    const auto result = configure();
    configuring_ = false;
    return result;
  }

  CallbackReturn configure()
  {
    reset();
    const auto cmd_vel_in_topic = get_or<std::string>("cmd_vel_in_topic", "cmd_vel_smoothed");
    const auto cmd_vel_out_topic = get_or<std::string>("cmd_vel_out_topic", "cmd_vel");
    const auto state_topic = get_or<std::string>("state_topic", "");
    const bool stamped = get_or<bool>("enable_stamped_cmd_vel", false);
    base_frame_id_ = get_or<std::string>("base_frame_id", "base_footprint");
    odom_frame_id_ = get_or<std::string>("odom_frame_id", "odom");
    base_shift_correction_ = get_or<bool>("base_shift_correction", true);
    // N26: converted as upstream converts it, by the same function.
    tolerance_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      tf2::durationFromSec(get_or<double>("transform_tolerance", 0.1))).count();
    refuse_unsupported_node_parameters();

    CollisionMonitor::Builder builder;
    stop_pub_timeout_ = get_or<double>("stop_pub_timeout", 1.0);
    builder.set_stop_pub_timeout(stop_pub_timeout_);
    node_source_timeout_ = get_or<double>("source_timeout", 2.0);
    const std::string scan_topic = read_source(builder);
    read_zones(builder);

    CollisionMonitor monitor = builder.build();
    if (!monitor.ok()) {
      // The core refuses the configuration. Ask it why and name the
      // parameters.
      monitor_.emplace(std::move(monitor));
      explain_refusal();
      monitor_.reset();
    }
    if (!refusals_.empty()) {
      RCLCPP_ERROR(get_logger(), "%s", refusal_message().c_str());
      reset();
      return CallbackReturn::FAILURE;
    }
    monitor_.emplace(std::move(monitor));
    warn_about_what_protects_nothing();
    source_timeout_zero_ =
      CollisionMonitor::source_timeout_ns(node_source_timeout_, own_source_timeout_) ==
      std::optional<int64_t>(0);

    // The TF buffer and its listener, as upstream creates them (N1, N3): the
    // listener has a thread of its own, so transforms arrive while a
    // callback waits for one.
    //
    // The buffer gets a clock of its own, not the node's. tf2 uses the clock
    // it is given for two things: for its own waiting loop, which this node
    // does not call (wait_for), and to clear itself when that clock jumps
    // back. With the node's clock the clearing would run when rclcpp handles
    // the /clock message, which in this node is after the current callback,
    // not during it (there is no clock thread, D15); a transform that
    // arrived between the jump and the end of a waiting callback would be
    // forgotten where upstream keeps it. The node clears the buffer itself,
    // on the /clock message that shows the jump (on_clock; N28). Without
    // simulated time neither clock ever jumps.
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    tf_buffer_->setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    if (stamped) {
      pub_stamped_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_out_topic, qos);
      cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        cmd_vel_in_topic, qos,
        [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
          geometry_msgs::msg::TwistStamped out;
          const auto decision = on_cmd(msg->twist, out.twist);
          if (decision.velocity) {
            out.header = msg->header;
            pub_stamped_->publish(out);
          }
          if (decision.processed) {
            publish_polygons();
          }
        });
    } else {
      pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out_topic, qos);
      cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_in_topic, qos,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          geometry_msgs::msg::Twist out;
          const auto decision = on_cmd(*msg, out);
          if (decision.velocity) {
            pub_->publish(out);
          }
          if (decision.processed) {
            publish_polygons();
          }
        });
    }
    if (!state_topic.empty()) {
      state_pub_ = create_publisher<nav2_msgs::msg::CollisionMonitorState>(state_topic, qos);
    }
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {on_scan(msg);});
    for (uint32_t k = 0; k < zones_.size(); ++k) {
      auto & zone = zones_[k];
      if (zone.topic) {
        // N31: upstream's QoS.
        rclcpp::QoS polygon_qos = rclcpp::SystemDefaultsQoS();
        if (zone.transient_local) {
          polygon_qos.transient_local();
        }
        zone.sub = create_subscription<geometry_msgs::msg::PolygonStamped>(
          zone.sub_topic, polygon_qos,
          [this, k](geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg) {on_shape(k, msg);});
      }
      if (zone.visualize) {
        zone.pub = create_publisher<geometry_msgs::msg::PolygonStamped>(
          zone.pub_topic, rclcpp::SystemDefaultsQoS());
      }
    }
    // D15: under simulated time the node looks at /clock itself, beside
    // rclcpp's own subscription, to know whether the message received last
    // is one that rclcpp can use, and to see the clock advance while a
    // callback waits for a transform (N26; see wait_for).
    sim_time_ = get_or<bool>("use_sim_time", false);
    if (sim_time_) {
      clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::ClockQoS(),
        [this](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {(void)on_clock(*msg);});
    }

    monitor_->on_lifecycle(CollisionMonitor::Transition::Configure);
    RCLCPP_INFO(
      get_logger(), "Monitoring '%s' -> '%s' (%s) with scans from '%s' and %zu zone(s).",
      cmd_vel_in_topic.c_str(), cmd_vel_out_topic.c_str(),
      stamped ? "TwistStamped" : "Twist", scan_topic.c_str(), zones_.size());
    return CallbackReturn::SUCCESS;
  }

  /// N5. The publishers are activated, every visualised zone is published
  /// once, and the bond is created. The transition itself publishes no
  /// velocity and no state message.
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    for_each_publisher([](auto & p) {p.on_activate();});
    monitor_->on_lifecycle(CollisionMonitor::Transition::Activate);
    publish_polygons();
    createBond();
    return CallbackReturn::SUCCESS;
  }

  /// N6.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    monitor_->on_lifecycle(CollisionMonitor::Transition::Deactivate);
    for_each_publisher([](auto & p) {p.on_deactivate();});
    destroyBond();
    return CallbackReturn::SUCCESS;
  }

  /// N7. Subscriptions, publishers, the TF buffer, the stored scan and the
  /// stored shapes go, and so does the core's monitor: the next configure
  /// builds a new one from the parameters, in the state the model has after
  /// `cleanup` (no scan, no shapes).
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    if (monitor_) {
      monitor_->on_lifecycle(CollisionMonitor::Transition::Cleanup);
    }
    reset();
    return CallbackReturn::SUCCESS;
  }

  /// N8, ND4. After this the core answers nothing, whatever arrives.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    if (monitor_) {
      monitor_->on_lifecycle(CollisionMonitor::Transition::Shutdown);
    }
    return CallbackReturn::SUCCESS;
  }

private:
  /// Back to what the node holds while unconfigured.
  void reset()
  {
    cmd_sub_.reset();
    scan_sub_.reset();
    clock_sub_.reset();
    pub_.reset();
    pub_stamped_.reset();
    state_pub_.reset();
    zones_.clear();
    tf_listener_.reset();
    tf_buffer_.reset();
    scan_.reset();
    monitor_.reset();
    refusals_.clear();
    refused_names_.clear();
    own_source_timeout_.reset();
    logged_ = false;
    cost_warned_readings_ = 0;
  }

  template<typename F>
  void for_each_publisher(F f)
  {
    if (pub_) {f(*pub_);}
    if (pub_stamped_) {f(*pub_stamped_);}
    if (state_pub_) {f(*state_pub_);}
    for (auto & zone : zones_) {
      if (zone.pub) {f(*zone.pub);}
    }
  }

  // ---- the clock ----------------------------------------------------------

  void set_clock_valid(bool valid)
  {
    if (valid == clock_valid_) {
      return;
    }
    clock_valid_ = valid;
    if (valid) {
      RCLCPP_INFO(get_logger(), "The clock is valid again: commands are evaluated as usual.");
    } else {
      RCLCPP_ERROR(
        get_logger(), "The node has no valid clock reading: a message on /clock carries a "
        "negative time, or an exception left a callback. Until a usable /clock message arrives "
        "every command is answered with zero velocity (D15).");
    }
  }

  /// A /clock message, taken by the executor or, while a callback waits for
  /// a transform, by hand (clock_while_waiting). Returns whether it is one
  /// rclcpp can use (D15).
  ///
  /// N28: a message with an earlier time than the one before it is a
  /// backward jump of the clock, and the TF buffer is cleared, as tf2 does
  /// (tf2_ros::Buffer::onTimeJump: every backward jump). Upstream's buffer
  /// is cleared by rclcpp's clock thread the moment the message arrives,
  /// also while a callback waits. This node has no clock thread (D15), and
  /// rclcpp's own handling of the message runs only after the callback has
  /// returned, so the buffer is not attached to the node clock (see where it
  /// is created) and is cleared here.
  bool on_clock(const rosgraph_msgs::msg::Clock & msg)
  {
    const bool usable = clock_message_usable(msg.clock.sec);
    set_clock_valid(usable);
    if (usable) {
      const int64_t t = static_cast<int64_t>(msg.clock.sec) * 1000000000 + msg.clock.nanosec;
      if (t < sim_clock_ns_ && tf_buffer_) {
        RCLCPP_WARN(get_logger(), "Detected jump back in time. Clearing TF buffer.");
        tf_buffer_->clear();
      }
      sim_clock_ns_ = t;
    }
    return usable;
  }

  // One clock reading per event (D7, ND2). The core takes unsigned
  // nanoseconds.
  uint64_t now_ns()
  {
    return static_cast<uint64_t>(std::max<rcl_time_point_value_t>(now().nanoseconds(), 0));
  }

  /// The node clock as far as it can be known inside a callback. Under
  /// simulated time rclcpp's clock stands still while a callback runs
  /// (there is no clock thread, D15), so the node takes the /clock messages
  /// of its own subscription by hand while it waits. The reading is the time
  /// of the newest usable message taken by hand in this callback, and
  /// `callback_start` (rclcpp's reading when the callback began) as long as
  /// there is none. It can go backwards: that is how a reset of the
  /// simulation shows (wait_for).
  int64_t clock_while_waiting(int64_t callback_start)
  {
    if (!sim_time_) {
      return now().nanoseconds();
    }
    if (clock_sub_) {
      rosgraph_msgs::msg::Clock msg;
      rclcpp::MessageInfo info;
      while (clock_sub_->take(msg, info)) {
        if (on_clock(msg)) {
          callback_clock_ns_ = sim_clock_ns_;
        }
      }
    }
    return callback_clock_ns_.value_or(callback_start);
  }

  // ---- transforms (T6) ----------------------------------------------------

  /// N26: the lookup, with upstream's wait, which is tf2's
  /// (tf2_ros::Buffer::canTransform with a timeout; it is not called,
  /// because it reads a clock that cannot advance here under simulated
  /// time). If the transform cannot be had at once, the callback sleeps
  /// 10 ms of wall time at a time, trying again each time, while
  ///   - the node clock is before `start + transform_tolerance`, and
  ///   - the node clock is not more than 3 s before `start` (tf2: "don't
  ///     wait, bag loop detected"): after a backward jump of the clock by
  ///     more than 3 s the wait ends at once (review CM5, finding 2; without
  ///     this the node waited until the clock was back at `start +
  ///     transform_tolerance`, handling nothing meanwhile),
  /// and tries once more at the end. Nothing else is handled meanwhile, as
  /// upstream. The wait also ends if the clock stops being valid (D15).
  static constexpr int64_t kBagLoopNs = 3000000000;  // tf2's rclcpp::Duration(3, 0)

  /// tf2's two conditions on the clock for going on waiting.
  bool keep_waiting(int64_t clock_ns, int64_t start_ns) const
  {
    return wait_goes_on(clock_ns, start_ns, tolerance_ns_, kBagLoopNs);
  }

  std::optional<tf2::Transform> wait_for(const Request & request, int64_t start_ns)
  {
    std::string why;
    auto tf = try_lookup(*tf_buffer_, request, &why);
    bool slept = false;
    while (!tf && rclcpp::ok() && clock_valid_ &&
      keep_waiting(clock_while_waiting(start_ns), start_ns))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      slept = true;
      tf = try_lookup(*tf_buffer_, request, &why);
    }
    if (slept) {
      // Once more at the end, as tf2 does; and the clock the wait ended with
      // is kept for the stamp of the polygons (publish_polygons).
      if (!tf && clock_valid_) {
        tf = try_lookup(*tf_buffer_, request, &why);
      }
      waited_until_ns_ = clock_while_waiting(start_ns);
    }
    if (!tf) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Failed to get the transform from '%s' to '%s': %s",
        request.source.c_str(), request.target.c_str(), why.c_str());
    }
    return tf;
  }

  // ---- events ------------------------------------------------------------

  void on_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    if (!monitor_) {
      return;
    }
    const uint64_t t = now_ns();
    const int64_t stamp_ns =
      static_cast<int64_t>(msg->header.stamp.sec) * 1000000000 + msg->header.stamp.nanosec;
    warn_about_cost(msg->ranges.size());
    const auto ranges = encode_ranges(msg->ranges);
    monitor_->on_scan(
      t, stamp_ns, bits_of(msg->range_min), bits_of(msg->range_max), ranges.data(),
      ranges.size());
    // T2 (d): the message whose ranges the core has just been given is the
    // one whose points are handed over with the following commands.
    scan_ = msg;
    scan_stamp_ns_ = stamp_ns;
  }

  /// N33, ND6. A polygon message for zone `k`. With at least three points its
  /// transform is looked up (with the wait of N26) and its vertices are
  /// handed over in the base frame; with fewer the core is given the short
  /// list at once and ignores it. What the shim keeps follows the core's
  /// answer: the message, if it is the zone's shape now; nothing, if the
  /// core found a vertex it cannot use (the zone then has no shape and
  /// demands its action until a usable message arrives, ND6); what it had,
  /// if the core ignored the message.
  void on_shape(uint32_t k, geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg)
  {
    if (!monitor_ || k >= zones_.size()) {
      return;
    }
    const auto & points = msg->polygon.points;
    const Request request = shape_request(base_frame_id_, msg->header.frame_id);
    std::optional<tf2::Transform> tf = identity_transform();
    callback_clock_ns_.reset();
    if (points.size() >= 3) {
      tf = wait_for(request, static_cast<int64_t>(now_ns()));
    }
    const auto verts = tf ? polygon_vertices(*tf, points) : std::vector<uint8_t>();
    switch (monitor_->on_shape(k, !request.identity, tf.has_value(), verts.data(), verts.size())) {
      case CollisionMonitor::ShapeResult::Shape:
        zones_[k].shape = msg;
        zones_[k].display = *msg;
        break;
      case CollisionMonitor::ShapeResult::Unusable:
        // Nothing to look up for later commands and nothing to display: an
        // empty polygon in the base frame, as before the first message.
        zones_[k].shape.reset();
        zones_[k].display = geometry_msgs::msg::PolygonStamped();
        zones_[k].display.header.frame_id = base_frame_id_;
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "[%s]: polygon message with %zu point(s) in frame '%s' has a coordinate that is not "
          "finite or is not within 999.997 m in the base frame. The zone has no shape now and "
          "%s every command until a usable polygon message arrives (ND6)",
          zones_[k].name.c_str(), points.size(), msg->header.frame_id.c_str(),
          zones_[k].slowdown ? "slows down" : "stops");
        break;
      case CollisionMonitor::ShapeResult::Ignored:
        RCLCPP_ERROR(
          get_logger(), "[%s]: polygon message with %zu point(s) in frame '%s' is ignored: it "
          "needs at least three points and a transform to '%s' (N33); the zone keeps what it had",
          zones_[k].name.c_str(), points.size(), msg->header.frame_id.c_str(),
          base_frame_id_.c_str());
        break;
    }
  }

  struct Published
  {
    bool velocity;   ///< `out` is to be published
    bool processed;  ///< the command was processed: the polygons are published (N16)
  };

  /// Hands the command to the core, publishes the state message the core
  /// asks for, and fills `out` with the velocity to publish, if any.
  Published on_cmd(const geometry_msgs::msg::Twist & cmd, geometry_msgs::msg::Twist & out)
  {
    if (!monitor_) {
      return {false, false};
    }
    const uint64_t lx = bits_of(cmd.linear.x), ly = bits_of(cmd.linear.y),
      lz = bits_of(cmd.linear.z), ax = bits_of(cmd.angular.x), ay = bits_of(cmd.angular.y),
      az = bits_of(cmd.angular.z);
    const uint64_t t = now_ns();
    waited_until_ns_.reset();
    callback_clock_ns_.reset();
    std::optional<std::vector<uint8_t>> points;
    std::vector<uint8_t> zone_verts;
    if (clock_valid_) {
      // Which transforms upstream would look up for this command is the
      // core's answer (N22, N34): the shim neither waits where upstream
      // answers at once nor skips a lookup.
      const auto plan = monitor_->cmd_plan(t, lx, ly, lz, ax, ay, az);
      bool zones = plan != CollisionMonitor::Plan::None;
      if (plan == CollisionMonitor::Plan::ScanAndZones && scan_) {
        const auto tf = wait_for(
          scan_request(
            base_frame_id_, scan_->header.frame_id, scan_stamp_ns_, static_cast<int64_t>(t),
            odom_frame_id_, base_shift_correction_),
          static_cast<int64_t>(t));
        if (tf) {
          points = scan_points(*tf, scan_->angle_min, scan_->angle_increment, scan_->ranges);
        } else if (!source_timeout_zero_) {
          // The source is invalid: upstream does not reach its zone loop
          // (N34), and the core leaves the shapes as they are.
          zones = false;
        }
      }
      if (zones) {
        zone_verts = zone_vertices(static_cast<int64_t>(t));
      }
    }
    // D15: without a valid clock reading (also if the clock stopped being
    // valid during a wait) there is no time to hand over, and the core is
    // told so; it answers with a stop.
    const auto decision = clock_valid_ ?
      monitor_->on_cmd(
      t, lx, ly, lz, ax, ay, az, points.has_value(), points ? points->data() : nullptr,
      points ? points->size() : 0, zone_verts.data(), zone_verts.size()) :
      monitor_->on_cmd_no_clock(lx, ly, lz, ax, ay, az);
    log_change(decision);
    // N13: the state message, before the velocity.
    publish_state(decision);
    // What to publish for the answer is apply_answer (conversions.hpp, with
    // its own test), which takes the kind as the number the core returned.
    static_assert(
      static_cast<uint8_t>(CollisionMonitor::OutputKind::Silent) == 0 &&
      static_cast<uint8_t>(CollisionMonitor::OutputKind::Pass) == 1 &&
      static_cast<uint8_t>(CollisionMonitor::OutputKind::Slowdown) == 2 &&
      static_cast<uint8_t>(CollisionMonitor::OutputKind::StopZone) == 3 &&
      static_cast<uint8_t>(CollisionMonitor::OutputKind::StopSource) == 4 &&
      static_cast<uint8_t>(CollisionMonitor::OutputKind::Withheld) == 5,
      "OutputKind must have the codes of roslean_cm_out_kind");
    const bool processed = decision.kind != CollisionMonitor::OutputKind::Silent;
    std::array<double, 3> planar{};
    const Answer answer = apply_answer(
      static_cast<uint8_t>(decision.kind), decision.ratio_bits,
      {cmd.linear.x, cmd.linear.y, cmd.angular.z}, planar);
    if (answer == Answer::Nothing) {
      // Not processed (not active, N4; a component is not finite, R2), or
      // processed with the zero withheld (R15).
      RCLCPP_DEBUG(get_logger(), "Command not answered with a velocity");
      return {false, processed};
    }
    if (answer == Answer::PublishUnknownKind) {
      // Unreachable: the kind is `kindOf` of the core's output
      // (core/ROSleanCore/CollisionMonitorFFI.lean), a total function whose
      // values are 0 to 5. Were a core and a shim of different versions ever
      // linked together, the answer the shim does not understand is treated
      // as a stop, not as silence: `planar` is all zeros.
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "unknown output kind %u from the core: publishing zero velocity",
        static_cast<unsigned>(decision.kind));
    }
    // `out` is a new message: linear.z, angular.x and angular.y stay 0 (R16).
    out.linear.x = planar[0];
    out.linear.y = planar[1];
    out.angular.z = planar[2];
    return {true, processed};
  }

  /// The zone vertices of a command (SPEC_NAV2, 8.3): for every zone in
  /// order a count and the vertices of its stored message in the base frame
  /// under the newest transform, or "none" (count 0xFFFFFFFF). A lookup is
  /// made, with the wait of N26, for every enabled zone whose stored message
  /// is in another frame (N34, ND3). The array ends after the last zone that
  /// has vertices: a zone it does not reach has none.
  std::vector<uint8_t> zone_vertices(int64_t t)
  {
    std::vector<uint8_t> out;
    size_t used = 0;
    for (const auto & zone : zones_) {
      const size_t at = out.size();
      out.resize(at + 4, 0xFF);
      if (!zone.topic || !zone.enabled || !zone.shape) {
        continue;
      }
      const Request request = shape_request(base_frame_id_, zone.shape->header.frame_id);
      if (request.identity) {
        continue;
      }
      const auto tf = wait_for(request, clock_valid_ ? clock_while_waiting(t) : t);
      if (!tf) {
        continue;
      }
      const auto verts = polygon_vertices(*tf, zone.shape->polygon.points);
      put_le(out.data() + at, zone.shape->polygon.points.size(), 4);
      out.insert(out.end(), verts.begin(), verts.end());
      used = out.size();
    }
    out.resize(used);
    return out;
  }

  /// N13 (T8): the state message the core asks for. The names are the
  /// shim's table from zone index to name.
  void publish_state(const CollisionMonitor::Output & d)
  {
    if (!state_pub_) {
      return;
    }
    nav2_msgs::msg::CollisionMonitorState state;
    switch (d.report) {
      case CollisionMonitor::Report::None:
        return;
      case CollisionMonitor::Report::Normal:
        state.action_type = nav2_msgs::msg::CollisionMonitorState::DO_NOTHING;
        state.polygon_name = "";
        break;
      case CollisionMonitor::Report::Stop:
        state.action_type = nav2_msgs::msg::CollisionMonitorState::STOP;
        state.polygon_name = zone_name(d.report_zone);
        break;
      case CollisionMonitor::Report::Slowdown:
        state.action_type = nav2_msgs::msg::CollisionMonitorState::SLOWDOWN;
        state.polygon_name = zone_name(d.report_zone);
        break;
      case CollisionMonitor::Report::Source:
        state.action_type = nav2_msgs::msg::CollisionMonitorState::STOP;
        state.polygon_name = "invalid source";
        break;
      default:
        // Unreachable (`reportOf` has the values 0 to 4). An unknown report
        // is not published as something it is not.
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "unknown report %u from the core: no state message",
          static_cast<unsigned>(d.report));
        return;
    }
    state_pub_->publish(state);
  }

  /// N16 (T8): every visualised zone, with the node clock as the stamp.
  /// Under simulated time, after a callback that waited for a transform,
  /// rclcpp's clock still shows the time before the wait; the stamp is then
  /// the clock reading the wait ended with, which is what upstream's clock
  /// shows at this point.
  void publish_polygons()
  {
    rclcpp::Time stamp = now();
    if (sim_time_ && waited_until_ns_) {
      // Later than rclcpp's reading after an ordinary wait; earlier after a
      // wait in which the clock jumped back.
      stamp = rclcpp::Time(*waited_until_ns_, stamp.get_clock_type());
    }
    waited_until_ns_.reset();
    for (auto & zone : zones_) {
      if (zone.pub) {
        zone.display.header.stamp = stamp;
        zone.pub->publish(zone.display);
      }
    }
  }

  /// Logs the action the core demands when it differs from the one logged
  /// last, as upstream does on each change of its action (and in upstream's
  /// words). The text is built from the three numbers the core returned:
  /// the kind, the zone index and the ratio's bit pattern; they are compared
  /// as integers with those of the last logged answer. A command the core
  /// leaves unanswered is not a change. Nothing here feeds back into what is
  /// published.
  void log_change(const CollisionMonitor::Output & d)
  {
    if (d.kind == CollisionMonitor::OutputKind::Silent ||
      d.kind == CollisionMonitor::OutputKind::Withheld)
    {
      return;
    }
    if (logged_ && logged_kind_ == d.kind && logged_zone_ == d.zone &&
      logged_ratio_bits_ == d.ratio_bits)
    {
      return;
    }
    logged_ = true;
    logged_kind_ = d.kind;
    logged_zone_ = d.zone;
    logged_ratio_bits_ = d.ratio_bits;
    switch (d.kind) {
      case CollisionMonitor::OutputKind::Pass:
        RCLCPP_INFO(get_logger(), "Robot to continue normal operation");
        break;
      case CollisionMonitor::OutputKind::Slowdown:
        RCLCPP_INFO(
          get_logger(), "Robot to slowdown for %f percents due to %s polygon",
          double_of_bits(d.ratio_bits) * 100, zone_name(d.zone).c_str());
        break;
      case CollisionMonitor::OutputKind::StopZone:
        RCLCPP_INFO(get_logger(), "Robot to stop due to %s polygon", zone_name(d.zone).c_str());
        break;
      case CollisionMonitor::OutputKind::StopSource:
        RCLCPP_WARN(
          get_logger(), "Robot to stop due to invalid source: no scan was received yet, or the "
          "last one is older than the source timeout, or its transform to the base frame could "
          "not be obtained");
        break;
      default:
        break;
    }
  }

  /// COMPARISON.md, defect 2: a command tests every stored reading against
  /// every zone, in exact integer arithmetic, and takes time in proportion.
  /// The budget is 150 us per command at 1080 readings; what a command is
  /// estimated to cost with the configured zones and a scan of this size is
  /// said once (and again if a larger scan arrives) when it exceeds
  /// `kCostWarnNs`. The estimate is the measured cost on one machine, back
  /// to back (Apple M5: 25 ns per reading, plus 1.5 ns per reading and circle
  /// and 2 ns per reading and polygon vertex), doubled; an idle or slower
  /// processor takes several times as long. It changes nothing the node
  /// does.
  static constexpr double kCostWarnNs = 1.0e6;  // 1 ms: 5 % of a 20 ms command period

  double estimated_cmd_cost_ns(size_t readings) const
  {
    double per_reading = 25.0;
    for (const auto & zone : zones_) {
      if (zone.enabled) {
        // A topic zone has no vertices yet: counted as a square.
        per_reading += zone.circle ? 1.5 :
          2.0 * static_cast<double>(zone.topic ? 4 : zone.vertices.size());
      }
    }
    return 2.0 * per_reading * static_cast<double>(readings);
  }

  void warn_about_cost(size_t readings)
  {
    if (readings <= cost_warned_readings_) {
      return;
    }
    const double ns = estimated_cmd_cost_ns(readings);
    if (ns > kCostWarnNs) {
      cost_warned_readings_ = readings;
      RCLCPP_WARN(
        get_logger(), "A scan of %zu readings with the %zu configured zone(s): each command is "
        "estimated to take about %.1f ms in the core, more than %.1f ms. Commands are answered "
        "late, and scans and commands can queue behind them. The answers are not affected. "
        "Fewer readings or zones make it faster (docs/collision_monitor/COMPARISON.md, "
        "\"Cost of a command\").",
        readings, zones_.size(), ns / 1.0e6, kCostWarnNs / 1.0e6);
    }
  }

  std::string zone_name(uint32_t k) const
  {
    return k < zones_.size() ? zones_[k].name : "zone " + std::to_string(k);
  }

  // ---- parameters --------------------------------------------------------

  /// The value of a parameter, or nothing if it is not set. A value of the
  /// wrong type is refused, as upstream (R17).
  template<typename T>
  std::optional<T> get_opt(const std::string & name)
  {
    rclcpp::Parameter param;
    if (!is_set(name, &param)) {
      return std::nullopt;
    }
    try {
      return param.get_value<T>();
    } catch (const std::exception & e) {
      refuse(name, std::string("has the wrong type: ") + e.what());
      return std::nullopt;
    }
  }

  template<typename T>
  T get_or(const std::string & name, const T & fallback)
  {
    return get_opt<T>(name).value_or(fallback);
  }

  bool is_set(const std::string & name, rclcpp::Parameter * out = nullptr)
  {
    rclcpp::Parameter param;
    if (!has_parameter(name) || !get_parameter(name, param) ||
      param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      return false;
    }
    if (out) {
      *out = param;
    }
    return true;
  }

  /// Records a refusal. The node does not start if there is one; all of them
  /// are reported together (one per parameter: the first reason is kept).
  void refuse(const std::string & name, const std::string & why)
  {
    if (refused_names_.insert(name).second) {
      refusals_.push_back("parameter '" + name + "': " + why);
    }
  }

  bool refused(const std::string & name) const {return refused_names_.count(name) != 0;}

  std::string refusal_message() const
  {
    if (refusals_.empty()) {
      return "the Lean core does not accept the configuration";
    }
    std::string all = std::to_string(refusals_.size()) +
      (refusals_.size() == 1 ? " parameter refused: " : " parameters refused: ");
    for (size_t i = 0; i < refusals_.size(); ++i) {
      all += (i == 0 ? "" : "; ") + refusals_[i];
    }
    return all;
  }

  // What is still outside the subset, at node level. Everything refused is
  // reported in one message (refusal_message).
  void refuse_unsupported_node_parameters()
  {
    if (get_or<bool>("use_realtime_priority", false)) {
      refuse("use_realtime_priority", "real-time priority is not supported");
    }
  }

  /// Reads the one scan source and returns its topic.
  std::string read_source(CollisionMonitor::Builder & builder)
  {
    const auto sources = get_opt<std::vector<std::string>>("observation_sources");
    if (!sources.has_value()) {
      refuse("observation_sources", "required");
      return "scan";
    }
    if (sources->size() != 1) {
      refuse(
        "observation_sources", "exactly one source is supported; " +
        std::to_string(sources->size()) + " given");
      return "scan";
    }
    source_name_ = sources->front();
    const std::string & s = source_name_;
    const auto type = get_or<std::string>(s + ".type", "scan");
    if (type != "scan") {
      refuse(s + ".type", "only 'scan' sources are supported; got '" + type + "'");
    }
    const auto topic = get_or<std::string>(s + ".topic", "scan");
    source_enabled_ = get_or<bool>(s + ".enabled", true);
    // The source's own timeout, if set; otherwise the core derives it from
    // the node's, as upstream's default does (R17).
    own_source_timeout_ = get_opt<double>(s + ".source_timeout");
    builder.set_source(source_enabled_, node_source_timeout_, own_source_timeout_);
    // `<source>.min_height` and `max_height` are not read: upstream does not
    // read them for a scan source either (N21).
    if (is_set(s + ".roslean_sensor_pose")) {
      refuse(
        s + ".roslean_sensor_pose", "removed: the pose of the sensor comes from TF, as in "
        "nav2_collision_monitor. Publish the transform from the scan's frame to '" +
        base_frame_id_ + "' (for a fixed sensor: a static transform) and delete this parameter");
    }
    return topic;
  }

  void read_zones(CollisionMonitor::Builder & builder)
  {
    const auto names = get_opt<std::vector<std::string>>("polygons");
    if (!names.has_value()) {
      refuse("polygons", "required");
      return;
    }
    // ND5: a state message names a zone. Two zones of one name, a zone named
    // like the state of an invalid source, and a zone without a name could
    // not be told apart by a reader of the state topic.
    std::set<std::string> seen;
    for (const auto & z : *names) {
      if (z.empty()) {
        refuse("polygons", "a zone must have a name (ND5)");
      } else if (z == "invalid source") {
        refuse(
          "polygons", "a zone must not be named 'invalid source': the state topic uses that name "
          "for an invalid source (ND5)");
      } else if (!seen.insert(z).second) {
        refuse("polygons", "zone '" + z + "' is listed twice: zone names must differ (ND5)");
      }
    }
    for (const auto & z : *names) {
      read_zone(builder, z);
    }
  }

  // Zones are added in the order of `polygons`; that order is the zone index
  // in the core's answers. A zone that the shim refuses is not added (the
  // transition fails in that case).
  void read_zone(CollisionMonitor::Builder & builder, const std::string & z)
  {
    bool complete = true;
    const auto type = get_opt<std::string>(z + ".type");
    if (!type.has_value()) {
      refuse(z + ".type", "required ('circle' or 'polygon')");
      complete = false;
    } else if (*type != "circle" && *type != "polygon") {
      refuse(z + ".type", "only 'circle' and 'polygon' are supported; got '" + *type + "'");
      complete = false;
    }
    const auto action_type = get_opt<std::string>(z + ".action_type");
    if (!action_type.has_value()) {
      refuse(z + ".action_type", "required ('stop' or 'slowdown')");
      complete = false;
    } else if (*action_type != "stop" && *action_type != "slowdown") {
      refuse(
        z + ".action_type", "only 'stop' and 'slowdown' are supported; got '" + *action_type +
        "'. (To use Nav2's footprint zone, give it 'points', or 'action_type: stop' together "
        "with 'polygon_sub_topic'; 'footprint_topic' is read for 'approach' only.)");
      complete = false;
    }
    if (is_set(z + ".max_points")) {
      refuse(z + ".max_points", "deprecated and not supported; use min_points (D10)");
    }
    const auto sources_names = get_opt<std::vector<std::string>>(z + ".sources_names");
    if (sources_names.has_value() && *sources_names != std::vector<std::string>{source_name_}) {
      refuse(z + ".sources_names", "must be the one source '" + source_name_ + "' or unset");
    }
    ZoneParams zone;
    zone.name = z;
    zone.min_points = get_or<int64_t>(z + ".min_points", 4);
    zone.ratio = get_or<double>(z + ".slowdown_ratio", 0.5);
    zone.enabled = get_or<bool>(z + ".enabled", true);
    zone.slowdown = action_type.has_value() && *action_type == "slowdown";
    zone.visualize = get_or<bool>(z + ".visualize", false);
    zone.pub_topic = get_or<std::string>(z + ".polygon_pub_topic", z);
    zone.display.header.frame_id = base_frame_id_;
    const auto action = zone.slowdown ?
      CollisionMonitor::ZoneAction::Slowdown : CollisionMonitor::ZoneAction::Stop;

    if (type.has_value() && *type == "circle") {
      const auto radius = get_opt<double>(z + ".radius");
      if (!radius.has_value()) {
        refuse(
          z + ".radius", "required: a circle whose radius comes from a topic is not supported "
          "(ND12)");
        return;
      }
      if (!complete) {
        return;
      }
      zone.circle = true;
      zone.radius = *radius;
      for (const auto & v : circle_polygon(*radius)) {
        geometry_msgs::msg::Point32 p;
        p.x = v[0];
        p.y = v[1];
        zone.display.polygon.points.push_back(p);
      }
      builder.add_circle(*radius, action, zone.ratio, zone.min_points, zone.enabled);
    } else if (type.has_value() && *type == "polygon") {
      const auto points = get_opt<std::string>(z + ".points");
      if (points.has_value()) {
        // N29: with `points` the shape is fixed; `polygon_sub_topic` and
        // `footprint_topic` are not read.
        std::string error;
        if (!parse_points(*points, zone.vertices, error)) {
          refuse(z + ".points", error);
          return;
        }
        if (!complete) {
          return;
        }
        builder.add_polygon(action, zone.ratio, zone.min_points, zone.enabled);
        for (const auto & v : zone.vertices) {
          builder.add_vertex(v[0], v[1]);
          geometry_msgs::msg::Point32 p;
          p.x = v[0];
          p.y = v[1];
          zone.display.polygon.points.push_back(p);
        }
      } else {
        const auto sub_topic = get_opt<std::string>(z + ".polygon_sub_topic");
        if (!sub_topic.has_value() || sub_topic->empty()) {
          // N29, ND11: upstream fails here too.
          std::string why =
            "is not initialized, and the zone has no 'points': static points and sub topic both "
            "not defined";
          if (is_set(z + ".footprint_topic")) {
            why += ". 'footprint_topic' is read for 'action_type: approach' only, which this node "
              "does not provide; give the zone 'points' (recommended), or set "
              "'polygon_sub_topic' to the topic the footprint is published on";
          }
          refuse(z + ".polygon_sub_topic", why);
          return;
        }
        if (!complete) {
          return;
        }
        zone.topic = true;
        zone.sub_topic = *sub_topic;
        zone.transient_local = get_or<bool>(z + ".polygon_subscribe_transient_local", false);
        builder.add_topic_polygon(action, zone.ratio, zone.min_points, zone.enabled);
      }
    } else {
      return;
    }
    zones_.push_back(zone);
  }

  /// Why the core refuses the configuration: records a refusal for every
  /// parameter the core does not accept. The reasons are the core's (its
  /// conversions and its error codes); this only words them.
  void explain_refusal()
  {
    const char * timeout = "a timeout must be finite, not negative and below 9.2e9 s (D9)";
    // Each timeout is asked about on its own, so that all of them are named
    // and not only the first.
    if (!CollisionMonitor::timeout_ns(node_source_timeout_).has_value()) {
      refuse("source_timeout", timeout);
    }
    if (own_source_timeout_.has_value() &&
      !CollisionMonitor::timeout_ns(*own_source_timeout_).has_value())
    {
      refuse(source_name_ + ".source_timeout", timeout);
    }
    if (!CollisionMonitor::timeout_ns(stop_pub_timeout_).has_value()) {
      refuse("stop_pub_timeout", timeout);
    }
    for (uint32_t k = 0; k < zones_.size(); ++k) {
      const std::string & z = zones_[k].name;
      switch (monitor_->zone_error(k)) {
        case CollisionMonitor::ZoneError::None:
          break;
        case CollisionMonitor::ZoneError::Radius:
          refuse(
            z + ".radius", "must be finite and at most 999.999 m in magnitude: the zone is the "
            "radius rounded up to a whole millimetre plus 1 mm, and that must not exceed 1000 m "
            "(the double written 999.999 lies just above the bound and is refused; 999.9989 is "
            "accepted) (D4, D12)");
          break;
        case CollisionMonitor::ZoneError::Points:
          refuse(
            z + ".points", "a polygon needs at least four vertices, all finite and within "
            "999.997 m of both axes (R10, D12)");
          break;
        case CollisionMonitor::ZoneError::Ratio:
          refuse(z + ".slowdown_ratio", "must not be negative (D5)");
          break;
        case CollisionMonitor::ZoneError::MinPoints:
          refuse(z + ".min_points", "must be in the range of a 32-bit integer (D10)");
          break;
        case CollisionMonitor::ZoneError::NoSuchZone:
          refuse(z, "zone unknown to the core");
          break;
      }
    }
    if (refusals_.empty()) {
      // Not reachable through this file: the source is always set before the
      // zones, and vertices are added only after their polygon.
      refusals_.push_back(
        "the Lean core does not accept the configuration (code " +
        std::to_string(static_cast<unsigned>(monitor_->error())) + ")");
    }
  }

  /// Finding 7c of the review CM3/CM4: a configuration that is accepted but
  /// protects nothing, or less than it seems to, is said so at start-up.
  /// These are warnings: whether to refuse such configurations is an open
  /// decision (SPEC, "Open decisions"). Nothing here changes what the node
  /// does.
  void warn_about_what_protects_nothing()
  {
    const auto warn = [this](const std::string & what) {
        RCLCPP_WARN(get_logger(), "This configuration protects nothing here: %s", what.c_str());
      };
    if (zones_.empty()) {
      warn("'polygons' is empty: there is no zone, so no obstacle stops or slows the robot.");
    }
    if (!source_enabled_) {
      warn(
        "'" + source_name_ + ".enabled' is false: scans are not used, no zone can be triggered "
        "by an obstacle, and missing data does not stop the robot (R6).");
    } else if (CollisionMonitor::source_timeout_ns(node_source_timeout_, own_source_timeout_) ==
      std::optional<int64_t>(0))
    {
      warn(
        "the source timeout is 0: a scan is used however old it is, and with no scan, or "
        "without a transform for it, commands pass as if nothing were near (R7, R8, N27).");
    }
    for (const auto & zone : zones_) {
      const std::string z = "zone '" + zone.name + "': ";
      if (zone.min_points <= 0) {
        warn(
          z + "'min_points' is " + std::to_string(zone.min_points) + ": the zone triggers on " +
          "every command while it is enabled, with or without an obstacle (R11).");
      }
      if (zone.slowdown && !(zone.ratio < 1.0)) {
        warn(
          z + "'slowdown_ratio' is " + std::to_string(zone.ratio) + ", not below 1: the zone "
          "never slows the robot down (R13).");
      }
      if (zone.circle && zone.radius == 0.0) {
        warn(z + "'radius' is 0: the zone is a disc of 1 mm around the origin of the base frame.");
      }
      if (zone.topic) {
        warn(
          z + "its shape comes from the topic '" + zone.sub_topic + "': until the first message "
          "with at least three points is accepted the zone takes no action, and a shape given in "
          "another frame stays where it was last published (N32, N34).");
      }
      if (!zone.circle && !zone.topic) {
        // The vertices as the core sees them, in whole millimetres.
        std::vector<std::array<int64_t, 2>> mm;
        for (const auto & v : zone.vertices) {
          const auto x = CollisionMonitor::vertex_mm(v[0]);
          const auto y = CollisionMonitor::vertex_mm(v[1]);
          if (x.has_value() && y.has_value() && std::llabs(*x) < (1 << 20) &&
            std::llabs(*y) < (1 << 20))
          {
            mm.push_back({*x, *y});
          }
        }
        if (mm.size() == zone.vertices.size() && twice_area_mm2(mm) == 0) {
          warn(
            z + "the polygon of 'points' has no area (its vertices lie on one line, or repeat): "
            "only points within a few millimetres of that line count as inside.");
        }
      }
    }
  }

  std::optional<CollisionMonitor> monitor_;
  std::vector<ZoneParams> zones_;  // as read; index = the core's zone index
  std::vector<std::string> refusals_;  // every reason to fail `configure`
  std::set<std::string> refused_names_;
  // Raw values as read, for messages only.
  double node_source_timeout_{0.0};
  std::optional<double> own_source_timeout_;
  double stop_pub_timeout_{0.0};
  bool source_enabled_{true};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_;
  // The answer logged last (log_change). Not read by anything else.
  bool logged_{false};
  CollisionMonitor::OutputKind logged_kind_{CollisionMonitor::OutputKind::Silent};
  uint32_t logged_zone_{0};
  uint64_t logged_ratio_bits_{0};
  size_t cost_warned_readings_{0};  // the largest scan the cost warning was given for
  // D15. `clock_valid_` selects which of the core's two command calls is
  // made; it is false only while the /clock message received last carried a
  // negative time, or after an exception was caught in main() and until the
  // next usable /clock message. Always true without simulated time.
  bool configuring_{false};  // on_configure is running (see the parameter callback)
  bool sim_time_{false};
  bool clock_valid_{true};
  int64_t sim_clock_ns_{0};  // the usable /clock message seen last (for waits only)
  // The clock reading at which the last wait of the current callback ended,
  // if it waited: for the stamp of the polygons only.
  std::optional<int64_t> waited_until_ns_;
  // The newest usable /clock message taken by hand in the current callback
  // (clock_while_waiting), if any: the clock of its waits.
  std::optional<int64_t> callback_clock_ns_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  std::string source_name_;
  // Transforms (T6).
  std::string base_frame_id_;
  std::string odom_frame_id_;
  bool base_shift_correction_{true};
  int64_t tolerance_ns_{0};
  // The source timeout is 0, by the core's conversion: a failed lookup for
  // the scan then does not make the source invalid (N27), and upstream goes
  // on to the zones' lookups. Selects lookups; enters no decision.
  bool source_timeout_zero_{false};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // The scan handed to the core last (T2 d).
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  int64_t scan_stamp_ns_{0};
  rclcpp::SubscriptionBase::SharedPtr cmd_sub_;
  rclcpp::SubscriptionBase::SharedPtr scan_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_stamped_;
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::CollisionMonitorState>::SharedPtr state_pub_;
};

}  // namespace roslean_collision_monitor

#ifdef ROSLEAN_COLLISION_MONITOR_COMPONENT
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(roslean_collision_monitor::CollisionMonitorNode)
#else
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int result = EXIT_SUCCESS;
  try {
    roslean::initialize();
    // Single-threaded: the core processes one event at a time (A1), and the
    // Lean runtime is initialised for this thread only.
    auto node = std::make_shared<roslean_collision_monitor::CollisionMonitorNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    // D15: no topic content may end the node. An exception that leaves a
    // callback (rclcpp's own /clock callback throws on a negative time) ends
    // spin(); the message that caused it has been taken, so spinning again
    // goes on with the next one. Every caught exception is logged as an
    // error (on_callback_exception).
    while (rclcpp::ok()) {
      try {
        executor.spin();
      } catch (const std::exception & e) {
        node->on_callback_exception(e.what());
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("collision_monitor"), "%s", e.what());
    result = EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return result;
}
#endif
