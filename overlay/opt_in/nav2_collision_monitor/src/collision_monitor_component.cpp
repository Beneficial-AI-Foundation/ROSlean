// Copyright 2026 Beneficial AI Foundation
// Licensed under the Apache License, Version 2.0.
//
// The component of the ROSlean overlay package named nav2_collision_monitor.
//
// Nothing is implemented here. The node is the one of
// ros/roslean_collision_monitor, whose source is included below as it is.
// What this file adds is a second name for its class, so that the component
// can be loaded under the plugin name that Nav2's launch files give:
//
//   ComposableNode(package='nav2_collision_monitor',
//                  plugin='nav2_collision_monitor::CollisionMonitor', ...)
//
// The name `roslean_collision_monitor::CollisionMonitorNode`, which the
// included source registers, stays loadable from this library as well; only
// upstream's name is listed in the ament index for this package.
//
// roslean_collision_monitor does not register upstream's name itself: its
// tests load its component and upstream's into one process, and two libraries
// that register one class name there would collide.

#define ROSLEAN_COLLISION_MONITOR_COMPONENT
#include "collision_monitor_node.cpp"  // NOLINT(build/include)

namespace nav2_collision_monitor
{
using CollisionMonitor = roslean_collision_monitor::CollisionMonitorNode;
}  // namespace nav2_collision_monitor

RCLCPP_COMPONENTS_REGISTER_NODE(nav2_collision_monitor::CollisionMonitor)
