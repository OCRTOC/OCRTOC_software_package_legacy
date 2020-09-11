from nvidia/cudagl:10.2-devel-ubuntu18.04

ENV TZ=Asia/Shanghai
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# Install Ubuntu tools.
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone \
    && rm /etc/apt/sources.list.d/cuda.list \
    && rm /etc/apt/sources.list.d/nvidia-ml.list \
    && apt-get update \
    && apt-get install -y lsb-release psmisc tree vim net-tools iputils-ping wget git python3-pip libglm-dev \
    # Install gl/x package for visualization inside docker
    && apt-get install -y libglvnd0 libgl1 libglx0 libegl1 libxext6 libx11-6

# Install ROS.
SHELL ["/bin/bash", "-c"]
RUN sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt update \
    && apt install -y ros-melodic-desktop-full \
    && echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc \
    && source ~/.bashrc \
    && apt install -y python-rosdep \
    && echo "151.101.84.133  raw.githubusercontent.com" >> /etc/hosts \
    && rosdep init \
    && rosdep update

# Upgrade Gazebo.
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
    && apt-get update \
    && apt-get install -y gazebo9 \
    && apt upgrade -y libignition-math4

# Install ROS dependencies.
RUN pip3 install catkin_pkg numpy transforms3d rospkg \
    && apt-get install -y ros-melodic-ddynamic-reconfigure \
                          ros-melodic-gripper-action-controller \
                          ros-melodic-joint-trajectory-controller \
                          ros-melodic-industrial-robot-status-interface \
                          ros-melodic-moveit \
                          ros-melodic-ros-control \
                          ros-melodic-ros-controllers \
                          ros-melodic-ur-msgs

# Copy files.
COPY ocrtoc_materials /root/ocrtoc_materials
COPY workspace /workspace

# Install Sapien.
RUN wget -qO - https://packages.lunarg.com/lunarg-signing-key-pub.asc | apt-key add - \
    && wget -qO /etc/apt/sources.list.d/lunarg-vulkan-1.2.148-bionic.list https://packages.lunarg.com/vulkan/1.2.148/lunarg-vulkan-1.2.148-bionic.list \
    && apt update \
    && apt install -y vulkan-sdk libglew-dev \
    && cd /workspace/glfw \
    && mkdir build && cd build && cmake .. && make install \
    && cd /workspace/spdlog \
    && mkdir build && cd build && cmake .. && make install \
    && cd /workspace/sapien \
    && source /opt/ros/melodic/setup.bash \
    && mkdir build && cd build && cmake .. && make pysapien_ros1

# Others.
RUN cd /root \
    && mkdir -p ocrtoc_ws/src \
    && cd ocrtoc_ws \
    && source /opt/ros/melodic/setup.bash \
    && catkin_make \
    && echo "source /root/ocrtoc_ws/devel/setup.bash" >> ~/.bashrc \
    && rm -rf /var/lib/apt/lists/*
