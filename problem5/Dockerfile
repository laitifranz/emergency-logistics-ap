# downloaded from the official course resources

FROM ros:humble

SHELL ["/bin/bash", "-c"]

# Update to latest version
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ="Europe/London"
RUN apt-get update

# install ros package
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-desktop \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py

WORKDIR /root

RUN mkdir -p ~/plansys2_ws/src && \
    cd ~/plansys2_ws/src && \
    git clone https://github.com/IntelligentRoboticsLabs/ros2_planning_system.git

RUN cd ~/plansys2_ws && rosdep install -y -r -q --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}

RUN cd ~/plansys2_ws/src && \
    git clone https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples.git

RUN cd ~/plansys2_ws && rosdep install -y -r -q --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}

RUN apt-get install -y vim gdb terminator

RUN cd ~/plansys2_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install

CMD /bin/bash