# syntax=docker/dockerfile:1.7

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base-noble

ARG ROS_DISTRO
ARG PX4_COMMIT=7ab709c2029a1a2c945be08d53eae1097a7aa47f
ARG PX4_MSGS_COMMIT=ff7ae284c4b9cb1c39d182e9f1a1343b3817011e
ARG XRCE_AGENT_COMMIT=73622810d984349b80bbac0ef55fc0b694d62222
ARG BUILD_JOBS=2

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=${ROS_DISTRO} \
    PX4_DIR=/opt/PX4-Autopilot \
    XRCE_AGENT=/opt/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        curl \
        git \
        gosu \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-ros-gz \
        xauth \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/PX4/PX4-Autopilot.git ${PX4_DIR} \
    && git -C ${PX4_DIR} checkout ${PX4_COMMIT} \
    && git -C ${PX4_DIR} submodule update --init --recursive

COPY drone_bringup/px4 /tmp/drone-px4

RUN RUNS_IN_DOCKER=true USER=root ${PX4_DIR}/Tools/setup/ubuntu.sh --no-nuttx \
    && git -C ${PX4_DIR} apply /tmp/drone-px4/0001-offboard-fixed-wing-speed.patch \
    && git -C ${PX4_DIR} apply /tmp/drone-px4/0002-load-based-battery-simulator.patch \
    && git -C ${PX4_DIR} apply --unidiff-zero \
        /tmp/drone-px4/0003-gazebo-headless-rendering.patch \
    && install -m 0755 /tmp/drone-px4/4030_gz_standard_vtol_lidar \
        ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/4030_gz_standard_vtol_lidar \
    && sed -i '/4004_gz_standard_vtol/a 4030_gz_standard_vtol_lidar' \
        ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt \
    && make -C ${PX4_DIR} -j${BUILD_JOBS} px4_sitl_default \
    && rm -rf /tmp/drone-px4

RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
        /opt/Micro-XRCE-DDS-Agent \
    && git -C /opt/Micro-XRCE-DDS-Agent checkout ${XRCE_AGENT_COMMIT} \
    && cmake -S /opt/Micro-XRCE-DDS-Agent \
        -B /opt/Micro-XRCE-DDS-Agent/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DUAGENT_BUILD_TESTS=OFF \
    && cmake --build /opt/Micro-XRCE-DDS-Agent/build --parallel ${BUILD_JOBS}

RUN existing_user="$(getent passwd 1000 | cut -d: -f1)" \
    && if [[ -n "${existing_user}" ]]; then \
        existing_group="$(id -gn "${existing_user}")"; \
        usermod --login drone --home /home/drone --move-home "${existing_user}"; \
        if [[ "${existing_group}" != "drone" ]]; then \
            groupmod --new-name drone "${existing_group}"; \
        fi; \
        usermod --gid drone --shell /bin/bash drone; \
    else \
        groupadd --gid 1000 drone; \
        useradd --create-home --uid 1000 --gid drone --shell /bin/bash drone; \
    fi \
    && install -d -o drone -g drone /data/photos /data/recordings /home/drone/.gz

# Cache all external world assets in the image. The SDF keeps the Fuel URLs so
# native and container runs resolve the same versioned cache layout.
RUN HOME=/home/drone gosu drone gz fuel download \
        -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/Radio%20Tower \
    && HOME=/home/drone gosu drone gz fuel download \
        -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/Tower%20Crane \
    && HOME=/home/drone gosu drone gz fuel download \
        -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/Water%20Tower

RUN mkdir -p /opt/px4_msgs_ws/src \
    && git clone https://github.com/PX4/px4_msgs.git /opt/px4_msgs_ws/src/px4_msgs \
    && git -C /opt/px4_msgs_ws/src/px4_msgs checkout ${PX4_MSGS_COMMIT} \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon --log-base /opt/px4_msgs_ws/log build \
        --base-paths /opt/px4_msgs_ws/src \
        --build-base /opt/px4_msgs_ws/build \
        --install-base /opt/px4_msgs_ws/install \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY . /opt/drone_ws/src/drone-apf-navigation

RUN rosdep update \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source /opt/px4_msgs_ws/install/setup.bash \
    && rosdep install --from-paths /opt/drone_ws/src \
        --ignore-src --rosdistro ${ROS_DISTRO} --as-root pip:false -r -y \
    && colcon --log-base /opt/drone_ws/log build \
        --base-paths /opt/drone_ws/src/drone-apf-navigation \
        --build-base /opt/drone_ws/build \
        --install-base /opt/drone_ws/install \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /var/lib/apt/lists/*

COPY docker/drone-dev-build /usr/local/bin/drone-dev-build
COPY docker/entrypoint.sh /usr/local/bin/drone-entrypoint

RUN chmod 0755 /usr/local/bin/drone-dev-build /usr/local/bin/drone-entrypoint

WORKDIR /workspace
EXPOSE 8765
ENTRYPOINT ["/usr/local/bin/drone-entrypoint"]
CMD ["run"]
