FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"] 

# Required ROS packages  
RUN sudo apt update && apt install -y ros-noetic-joy ros-noetic-teleop-twist-joy \
    ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
    ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
    ros-noetic-rosserial-python ros-noetic-rosserial-client \
    ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
    ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
    ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
    ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers \
    ros-noetic-vision-msgs ros-noetic-urg-node ros-noetic-rplidar-ros

# ROS packages for Turtlebot3 robot     
RUN sudo apt install -y ros-noetic-dynamixel-sdk \
    ros-noetic-turtlebot3-*

# We also install Git and other tools
RUN sudo apt install -y git vim nano iputils-ping net-tools

# MESA drivers for hardware acceleration graphics (Gazebo and RViz)
RUN sudo apt -y install libgl1-mesa-glx libgl1-mesa-dri

# Multiple terminals support (tmux)
RUN apt install -y tmuxinator

# By default the DockerFile uses the root as user. 
# We will add a custom user so that we work in user space by default
# And then switch to that user
# Source: https://www.docker.com/blog/understanding-the-docker-user-instruction/
RUN useradd -ms /bin/bash rssa
# Sometimes it is good to add a password so that we can use sudo to install things
# To this end, uncomment the following line, remove extra ### and change the password 
RUN echo "rssa:rssa" | chpasswd 
### Remove #### to change the password
RUN adduser rssa sudo & adduser rssa dialout & adduser rssa plugdev

USER rssa

# We will use the home directory as work directory
# So that the docker starts at HOME!
WORKDIR "/home/rssa"

