FROM osrf/ros:humble-desktop

COPY ./files/entrypoint.sh /
RUN chmod a+x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]



ARG WORKSPACE=humble_dev_ws
# Grab the cross build basics and uninstall the web browser to save ~ 250MB of space: 
RUN apt update &&\
    apt install -y \
        bindfs \
        git \
        libeigen3-dev \
        ninja-build \
        python3-dev \
        software-properties-common \
        sudo \
        unzip \
        wget \
        whois &&\
    rm -rf /var/lib/apt/lists/* &&\
    apt clean

RUN useradd -ms /bin/bash -G sudo -p `mkpasswd -m sha-512 ubuntu` ubuntu &&\
    chown ubuntu:ubuntu /home/ubuntu &&\
    adduser ubuntu dialout

# And use the .bashrc template plus our additions for the normal login shell
RUN cp /root/.bashrc /home/ubuntu/.bashrc &&\
    chown ubuntu.ubuntu /home/ubuntu/.bashrc

WORKDIR /home/ubuntu
 
ENV NVIDIA_VISIBLE_DEVICES \
     ${NVIDIA_VISIBLE_DEVICES:-all}
 ENV NVIDIA_DRIVER_CAPABILITIES \
     ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
#ENV QT_X11_NO_MITSHM=1
ENV EDITOR=code
# ENV XDG_RUNTIME_DIR=/tmp
# I removed gazebo
# I added foxglove-bridge to connect to foxglove
RUN apt-get update && apt-get install -y \
    cmake \
    curl \
    libglu1-mesa-dev \
    nano \
    python3-pip \
    python3-pydantic \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher \
    ros-humble-robot-localization \
    #ros-foxy-plotjuggler-ros \                  doesn't have in Humble
    ros-humble-robot-state-publisher \
    ros-humble-ros2bag \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rqt-tf-tree \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-msgs \
    ros-humble-twist-mux \
    ros-humble-usb-cam \
    ros-humble-joy-linux \
    ros-humble-xacro \
    ros-humble-foxglove-bridge \
    ruby-dev \
    rviz \
    tmux \
    wget \
    vim \
    xorg-dev \
    i2c-tools \
    libgpiod-dev \ 
    python3-libgpiod \
    ~nros-humble-rqt*

# VERY IMPORTANT, COLCON EATS YOUR MEMORY WITHOUT IT
RUN pip3 install setuptools==58.2.0

# install vscode
RUN apt update &&\
    apt install -y wget gpg &&\
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg &&\
    install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg &&\
    sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list' &&\
    rm -f packages.microsoft.gpg &&\
    apt install -y apt-transport-https &&\
    apt update &&\
    apt install -y code &&\
    rm -rf /var/lib/apt/lists/* &&\
    apt clean 

# RUN wget https://github.com/openrr/urdf-viz/releases/download/v0.38.2/urdf-viz-x86_64-unknown-linux-gnu.tar.gz && \
#     tar -xvzf urdf-viz-x86_64-unknown-linux-gnu.tar.gz -C /usr/local/bin/ && \
#     chmod +x /usr/local/bin/urdf-viz && \
#     rm -f urdf-viz-x86_64-unknown-linux-gnu.tar.gz
# 

RUN echo "source /opt/ros/humble/setup.bash" >> /home/ubuntu/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/ubuntu/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/ubuntu/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> /home/ubuntu/.bashrc
RUN echo "source /home/ubuntu/24-25WaterCode/scripts/command_aliases.sh" >> /home/ubuntu/.bashrc
RUN echo 'export PS1="\[\e[1;36m\]Fish\[\e[1;35m\]ROS \[\e[1;32m\]\w\[\e[0m\] -> "' >> /home/ubuntu/.bashrc

# RUN apt-get autoremove -y \
#     && apt-get clean -y \
#     && rm -rf /var/lib/apt/lists/*
# 
# RUN echo "export DISABLE_AUTO_TITLE=true" >> /root/.zshrc
# RUN echo 'LC_NUMERIC="en_US.UTF-8"' >> /root/.zshrc
# RUN echo "source /opt/ros/humble/setup.zsh" >> /root/.zshrc
# RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.zshrc
# 
# RUN echo 'alias rosdi="rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y"' >> /root/.zshrc
# RUN echo 'alias cbuild="colcon build --symlink-install"' >> /root/.zshrc
# RUN echo 'alias ssetup="source ./install/local_setup.zsh"' >> /root/.zshrc
# RUN echo 'alias cyclone="export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"' >> /root/.zshrc
# RUN echo 'alias fastdds="export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"' >> /root/.zshrc
# RUN echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /root/.zshrc
# 
# RUN echo "autoload -U bashcompinit" >> /root/.zshrc
# RUN echo "bashcompinit" >> /root/.zshrc
# RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> /root/.zshrc
# RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> /root/.zshrc

# RUN export DONT_PROMPT_WSL_INSTALL=1 &&\
#    code --install-extension ms-vscode.cpptools &&\
#    code --install-extension ms-vscode.cpptools-extension-pack &&\
#    code --install-extension ms-vscode.cpptools-themes &&\
#    code --install-extension ms-iot.vscode-ros &&\
#    code --install-extension ms-python.python &&\
#    code --install-extension ms-python.vscode-pylance

RUN rosdep update
