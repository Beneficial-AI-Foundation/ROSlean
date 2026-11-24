# The collision-monitor comparison (docs/collision_monitor/COMPARISON.md) on
# Linux: Ubuntu 24.04, ROS 2 Jazzy and nav2_collision_monitor from the ROS
# apt repository (not from pixi), the Lean core, the ROSlean packages and the
# benchmarks. Unlike docker/Dockerfile this image keeps the sources and the
# build tree, because the tests are run from them.
#
#   docker build -f bench/cm_linux/Dockerfile -t roslean-cm-linux .
#   docker run --rm roslean-cm-linux bash -c \
#     'cd ros/roslean_collision_monitor/test && python3 scenario.py \
#        roslean_collision_monitor/collision_monitor --profile profile_roslean'
#
# bench/cm_linux/run.sh (on the host) runs everything the document reports
# from Linux, one container per run.

FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      curl ca-certificates git procps \
      ros-jazzy-geometry-msgs ros-jazzy-std-msgs ros-jazzy-sensor-msgs \
      ros-jazzy-visualization-msgs ros-jazzy-diagnostic-updater \
      ros-jazzy-twist-mux ros-jazzy-nav2-collision-monitor \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSf https://elan.lean-lang.org/elan-init.sh | sh -s -- -y --default-toolchain none
ENV PATH=/root/.elan/bin:${PATH}

WORKDIR /src
COPY core core
RUN cd core && lake build

COPY ros ros
COPY bench/roslean_bench bench/roslean_bench
# As `pixi run bench-build`: no build type is passed.
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --base-paths ros bench --event-handlers console_direct+ > /src/build.log 2>&1 \
    || (tail -100 /src/build.log; false)

COPY docker/entrypoint.sh /entrypoint.sh
RUN sed -i 's#/opt/roslean/setup.bash#/src/install/setup.bash#' /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
