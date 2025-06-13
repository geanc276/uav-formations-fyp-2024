FROM ros:humble

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive




# 1. Install required packages along with Mesa OpenGL libraries and VNC components
RUN apt-get update && apt-get install -y \
    bat \
    wget \
    git \
    bash-completion \
    build-essential \
    sudo \
    libreadline-dev \
    ros-humble-control-toolbox \
    nlohmann-json3-dev \
    libtins-dev \
    libboost-all-dev \
    libtbb-dev \
    && rm -rf /var/lib/apt/lists/*

# 2. Set up user and groups
ARG UID=1000
ARG GID=1000

# Ensure the dialout group with GID=20 exists or create it
RUN if ! getent group 20 >/dev/null; then \
      addgroup --system --gid 20 dialout; \
    else \
      echo "dialout group (GID=20) already exists"; \
    fi

# Create user 'ros' with the appropriate group
RUN if [ "$GID" = "20" ]; then \
      adduser --gecos "ROS User" --disabled-password --uid "$UID" --gid 20 ros; \
    else \
      if ! getent group "$GID" >/dev/null; then \
        addgroup --gid "$GID" ros; \
      else \
        groupmod -n ros "$(getent group "$GID" | cut -d: -f1)"; \
      fi; \
      adduser --gecos "ROS User" --disabled-password --uid "$UID" --gid "$GID" ros; \
      usermod -a -G dialout ros; \
    fi

# Add sudo privileges for 'ros'
ADD configs/99_aptget /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

# Set environment variables
ENV USER=ros
ENV HOME=/home/ros
WORKDIR /home/ros

# Switch to user 'ros'
USER ros

# # 3. Set up ROS workspace
# RUN mkdir -p "$HOME/drones/uav-formations-fyp-2024/" && \
#     /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
#                   colcon build --symlink-install"

# Copy custom bashrc update script
USER root
COPY configs/update_bashrc.sh /sbin/update_bashrc.sh
RUN chmod +x /sbin/update_bashrc.sh && chown ros /sbin/update_bashrc.sh

USER ros
RUN /sbin/update_bashrc.sh && sudo rm /sbin/update_bashrc.sh

# Copy entrypoint script
USER root
COPY configs/entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh && chown ros /ros_entrypoint.sh
RUN chown ros /home/ros -R
# 5. Clean up (optional)
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# USER ros

USER ros

# 6. Entrypoint and default command
# ENTRYPOINT ["/ros_entrypoint.sh"]
# ENTRYPOINT ["/usr/bin/tini", "--"]
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
