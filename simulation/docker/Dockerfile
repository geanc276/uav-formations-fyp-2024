FROM ros:humble
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    git build-essential python3-colcon-common-extensions \
    ros-humble-gazebo-ros-pkgs ros-humble-control-toolbox \
    libreadline-dev libboost-all-dev libtbb-dev \
    sudo bash-completion x11-apps mesa-utils \
    && rm -rf /var/lib/apt/lists/*

ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID ros || true && useradd -m -u $UID -g $GID ros \
  && usermod -aG sudo,dialout ros \
  && echo 'ros ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ros \
  && chmod 0440 /etc/sudoers.d/ros

# -- IMPORTANT : rester root ici --
# 1) bashrc helper
COPY docker/configs/update_bashrc.sh /home/ros/update_bashrc.sh
RUN chown ros:ros /home/ros/update_bashrc.sh \
 && chmod +x /home/ros/update_bashrc.sh \
 && su - ros -c "/home/ros/update_bashrc.sh" \
 && rm /home/ros/update_bashrc.sh

# 2) entrypoint
COPY docker/configs/entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# -- maintenant seulement --
USER ros
WORKDIR /home/ros

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
