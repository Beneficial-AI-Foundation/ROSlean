# Lean-backed twist_mux and collision monitor on ROS 2 Jazzy (Ubuntu 24.04).
#
#   docker build -f docker/Dockerfile -t roslean-twist-mux .
#   docker run --rm --network host roslean-twist-mux
#
# Two stages. The builder has Lean (elan) and compiles everything; the final
# image has ROS, upstream twist_mux (for its helper nodes) and the install
# tree only: no Lean toolchain, no sources.
# `--build-arg RUN_TESTS=0` skips the tests (they take about 8 minutes and
# compare every decision with upstream twist_mux).

ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base AS toolchain
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      curl ca-certificates git \
      ros-${ROS_DISTRO}-geometry-msgs ros-${ROS_DISTRO}-std-msgs \
      ros-${ROS_DISTRO}-sensor-msgs ros-${ROS_DISTRO}-twist-mux \
      ros-${ROS_DISTRO}-rclcpp-lifecycle ros-${ROS_DISTRO}-rclcpp-components \
      ros-${ROS_DISTRO}-nav2-util ros-${ROS_DISTRO}-nav2-msgs \
      ros-${ROS_DISTRO}-tf2 ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-tf2-geometry-msgs \
      ros-${ROS_DISTRO}-tf2-msgs ros-${ROS_DISTRO}-lifecycle-msgs \
      ros-${ROS_DISTRO}-nav2-collision-monitor ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    && rm -rf /var/lib/apt/lists/*

# elan installs the toolchain pinned in lean-toolchain on first use.
RUN curl -sSf https://elan.lean-lang.org/elan-init.sh | sh -s -- -y --default-toolchain none
ENV PATH=/root/.elan/bin:${PATH}

WORKDIR /src
# The Lean core is a package without dependencies: this layer downloads the
# toolchain pinned in core/lean-toolchain and compiles the model to C. It is
# rebuilt only when the Lean sources change. Mathlib is never needed here;
# the proofs are checked separately (`lake build` in the repository root).
COPY core core
RUN cd core && lake build


FROM toolchain AS builder
ARG RUN_TESTS=1
COPY ros ros
# The image holds the two nodes. Left out: roslean_demos (it needs the
# simulators) and roslean_nav2_bringup (it needs nav2_bringup, which is not
# installed here).
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --base-paths ros --install-base /opt/roslean \
         --packages-skip roslean_demos roslean_nav2_bringup \
         --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN if [ "${RUN_TESTS}" = "1" ]; then \
      source /opt/ros/${ROS_DISTRO}/setup.bash \
      && colcon test --base-paths ros --install-base /opt/roslean \
           --packages-skip roslean_demos roslean_nav2_bringup \
           --event-handlers console_direct+ \
      && colcon test-result --verbose; \
    fi

# The package named twist_mux: the same node under upstream's package name,
# with upstream's launch and config file names (docs/DROP_IN_TWIST_MUX.md).
# Built after the tests on purpose: they compare with the package called
# twist_mux, which until here is upstream's from apt.
COPY overlay overlay
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && source /opt/roslean/setup.bash \
    && colcon build --base-paths overlay --build-base build_overlay \
         --install-base /opt/roslean --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && source /opt/roslean/setup.bash \
    && test "$(ros2 pkg prefix twist_mux)" = /opt/roslean/twist_mux \
    && test -x /opt/roslean/twist_mux/lib/twist_mux/twist_mux \
    && ros2 pkg executables twist_mux

# Fail the build if the install tree still refers to the Lean toolchain or to
# the source tree.
COPY docker/check_install.sh /tmp/check_install.sh
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && /tmp/check_install.sh /opt/roslean


FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-geometry-msgs ros-${ROS_DISTRO}-std-msgs \
      ros-${ROS_DISTRO}-sensor-msgs \
      ros-${ROS_DISTRO}-rclcpp-lifecycle ros-${ROS_DISTRO}-rclcpp-components \
      ros-${ROS_DISTRO}-nav2-util ros-${ROS_DISTRO}-nav2-msgs \
      ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-tf2-geometry-msgs \
      ros-${ROS_DISTRO}-nav2-lifecycle-manager \
      ros-${ROS_DISTRO}-twist-mux \
    && rm -rf /var/lib/apt/lists/*
COPY --from=builder /opt/roslean /opt/roslean
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# Mount your own parameter file and pass it instead to configure the inputs.
# The collision monitor is in the same image. It is a lifecycle node; with
# Nav2's lifecycle manager, which configures and activates it:
#   docker run --rm --network host roslean-twist-mux ros2 launch \
#     /opt/roslean/roslean_collision_monitor/share/roslean_collision_monitor/launch/collision_monitor_launch.py
# Upstream's twist_mux from apt is installed as well, for twist_marker and
# joystick_relay.py. The entrypoint sources /opt/roslean after /opt/ros, so
# `ros2 run twist_mux twist_mux` and `ros2 launch twist_mux twist_mux_launch.py`
# start the Lean-backed node; upstream's is /opt/ros/$ROS_DISTRO/lib/twist_mux/twist_mux.
CMD ["ros2", "run", "roslean_twist_mux", "twist_mux", "--ros-args", "--params-file", \
     "/opt/roslean/roslean_twist_mux/share/roslean_twist_mux/config/example.yaml"]
