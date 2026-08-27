FROM osrf/ros:jazzy-desktop

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

# Gazebo Harmonic (gz-sim 8) repo
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release ca-certificates \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Jazzy + Gazebo Harmonic integration + build tooling
RUN apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        ros-jazzy-ros-gz \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-image \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-ros2controlcli \
        ros-jazzy-xacro \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-rviz2 \
        ros-jazzy-topic-tools \
        ros-jazzy-nav2-map-server \
        ros-jazzy-nav2-lifecycle-manager \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        build-essential \
        git \
    && rm -rf /var/lib/apt/lists/*

# NVIDIA GLVND hints (consumed when runtime=nvidia)
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    QT_X11_NO_MITSHM=1

# Non-root user matching host uid/gid for clean bind-mount permissions
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

RUN set -eux; \
    EXISTING_USER="$(getent passwd "${USER_UID}" | cut -d: -f1 || true)"; \
    EXISTING_GROUP="$(getent group "${USER_GID}" | cut -d: -f1 || true)"; \
    \
    if [ -n "${EXISTING_GROUP}" ] && [ "${EXISTING_GROUP}" != "${USERNAME}" ]; then \
        groupmod -n "${USERNAME}" "${EXISTING_GROUP}"; \
    elif [ -z "${EXISTING_GROUP}" ]; then \
        groupadd --gid "${USER_GID}" "${USERNAME}"; \
    fi; \
    \
    if [ -n "${EXISTING_USER}" ] && [ "${EXISTING_USER}" != "${USERNAME}" ]; then \
        usermod -l "${USERNAME}" \
                -d "/home/${USERNAME}" \
                -m \
                -s /bin/bash \
                "${EXISTING_USER}"; \
        usermod -g "${USERNAME}" "${USERNAME}"; \
    elif [ -z "${EXISTING_USER}" ]; then \
        useradd --uid "${USER_UID}" \
                --gid "${USER_GID}" \
                -m \
                -s /bin/bash \
                "${USERNAME}"; \
    fi; \
    \
    apt-get update; \
    apt-get install -y --no-install-recommends sudo; \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}"; \
    chmod 0440 "/etc/sudoers.d/${USERNAME}"; \
    rm -rf /var/lib/apt/lists/*
USER ${USERNAME}
WORKDIR /home/${USERNAME}/swarm_ws

COPY --chown=${USERNAME}:${USERNAME} docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
