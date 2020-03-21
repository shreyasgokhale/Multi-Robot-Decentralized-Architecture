# This base image is used in simulatior as well as robots
# It must contain ROS + Gazebo

# For GPU acceleration, CUDA + ROS image has to be created

#FROM nvidia/cuda:10.1-runtime as nvidia_image

FROM ros:melodic
# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# setup sources.list
RUN . /etc/os-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-latest.list

# install gazebo packages
RUN apt-get update && apt-get install -q -y \
    gazebo9 \
    && rm -rf /var/lib/apt/lists/*

# install ros gazebo connection
RUN apt-get update &&  apt-get install -y ros-melodic-gazebo-ros-pkgs \
    ros-melodic-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

RUN  /bin/bash -c "source /opt/ros/melodic/setup.bash"
# setup environment
# EXPOSE 11345

# # setup entrypoint
# COPY ./gzserver_entrypoint.sh /

CMD ["/bin/bash", "-c", " roscore & rosrun gazebo_ros gazebo"]
