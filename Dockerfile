FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive


RUN apt-get update && apt-get install -y --fix-missing\
    locales \
    software-properties-common \
    curl \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && apt-get update 

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb

RUN apt-get update && apt-get install -y --fix-missing \
    ros-humble-desktop  \
    ros-dev-tools \
    libeigen3-dev \
    libyaml-cpp-dev \
    ros-humble-rviz-visual-tools \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen

RUN apt-get update && apt-get install -y  \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest-cov 

RUN apt-get update && apt-get install -y  \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pytest-repeat \
    python3-pytest-rerunfailures 

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /root/.bashrc