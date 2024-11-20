FROM osrf/ros:humble-desktop

# Update and install essential packages
RUN apt-get update \
  && apt-get install -y \
  wget \
  lsb-release \
  sudo \
  mesa-utils \
  && apt-get clean

# Get Gazebo binaries
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
  && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
  && apt-get update \
  && apt-get install -y \
  gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-joy \
  libignition-math6 \
  libignition-math6-dev \
  && apt-get clean

# Install rqt and related tools
RUN apt-get update \
  && apt-get install -y \
  ros-humble-rqt \
  ros-humble-rqt-graph \
  ros-humble-rqt-topic \
  && apt-get clean

# Set up workspace and build
RUN mkdir -p /tmp/workspace/src
COPY prius_description /tmp/workspace/src/prius_description
COPY prius_msgs /tmp/workspace/src/prius_msgs
COPY car_demo /tmp/workspace/src/car_demo
COPY data_generator /tmp/workspace/src/data_generator
RUN /bin/bash -c 'cd /tmp/workspace \
  && source /opt/ros/humble/setup.bash \
  && rosdep install -y --from-paths `colcon list --packages-up-to car_demo -p` --ignore-src \
  && colcon build'

# Set up entry point
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /tmp/workspace/install/setup.bash && ros2 launch car_demo demo.launch.py"]