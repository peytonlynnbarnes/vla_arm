FROM ros:jazzy-ros-base

ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        curl \
        git \
        less \
        python3-colcon-common-extensions \
        python3-opencv \
        python3-pip \
        python3-requests \
        python3-rosdep \
        ros-jazzy-joint-trajectory-controller \
        ros-jazzy-ros-gz \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-rviz2 \
        ros-jazzy-xacro \
        sudo \
        vim \
    && rm -rf /var/lib/apt/lists/*

# json-numpy isn't packaged as .deb; install via pip
RUN pip3 install --break-system-packages --no-cache-dir json-numpy

# Node.js + Claude Code CLI
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y nodejs \
    && rm -rf /var/lib/apt/lists/* \
    && npm install -g @anthropic-ai/claude-code

# Non-root user matching host UID/GID so files edited inside the
# container are owned by your host user on the bind-mounted volume.
# The ros:jazzy-ros-base image ships with an `ubuntu` user at UID/GID 1000;
# drop it first so we can reuse the target UID/GID.
RUN if getent passwd ${USER_UID} >/dev/null; then \
        userdel -r "$(getent passwd ${USER_UID} | cut -d: -f1)" 2>/dev/null || true; \
    fi \
 && if getent group ${USER_GID} >/dev/null; then \
        groupdel "$(getent group ${USER_GID} | cut -d: -f1)" 2>/dev/null || true; \
    fi \
 && groupadd --gid ${USER_GID} dev \
 && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash dev \
 && echo 'dev ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/dev

# Source ROS automatically for every bash invocation Claude makes
# (BASH_ENV is read by non-interactive shells, which `bash -c` is).
RUN printf '%s\n' \
        'source /opt/ros/jazzy/setup.bash' \
        '[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash' \
        > /etc/profile.d/ros.sh
ENV BASH_ENV=/etc/profile.d/ros.sh

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER dev
WORKDIR /workspace

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
