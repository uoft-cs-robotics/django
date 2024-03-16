ARG image=ubuntu:20.04
FROM ${image}

ARG DEBIAN_FRONTEND=noninteractive
ENV WS_DIR="/root"
WORKDIR ${WS_DIR}

SHELL ["/bin/bash", "-c"]
# Dependencies for latency plot
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    curl \
    grep \
    build-essential \
    make \
    libnuma-dev \
    python3 \
    python3-distutils \
    gnuplot-qt \ 
    sshpass \
    iproute2 \
    python3-pip \
    gnome-terminal\
    nano\
    vim \
    git

# Dependencies for Bash unit-tests
RUN apt-get update && apt-get install -y \
    bats \
    dialog \
    tmux \
    iputils-ping 	

# ROS Install begins
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    python3-catkin-tools \
    ros-noetic-jsk-tools \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && \
    rosdep update

RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /root/jango_ws/devel/setup.bash" >> ~/.bashrc
# ROS Install ends

ENV ROS_DISTRO noetic

RUN echo "copying jango ROS packages"
RUN mkdir /root/jango_ws/src -p \
    && source /opt/ros/noetic/setup.bash\ 
    && apt-get update && apt-get -y upgrade 

COPY ./jango_src /root/jango_ws/src
RUN apt-get install -y ros-noetic-ridgeback-desktop ros-noetic-catkin   ros-noetic-serial libserial-dev\ 
    &&  python3 -m pip install conan==1.59 \
    && conan config set general.revisions_enabled=1

RUN echo "Building jango_ws" 
RUN source /opt/ros/noetic/setup.bash && cd /root/jango_ws && rosdep install --from-paths src --ignore-src -r -y \
    && source /opt/ros/noetic/setup.bash\ 
    && catkin_make 

RUN echo "copying atom related ROS packages"
RUN mkdir /root/atom_ws/src -p \
    && source /opt/ros/noetic/setup.bash\ 
    && apt-get update && apt-get -y upgrade 
COPY ./atom_src /root/atom_ws/src 

RUN echo "Building atom_ws" \
    && cd /root/atom_ws/src/atom \
    && pip3 install -r requirements.txt \     
    && sudo apt-get install -y  ros-noetic-ros-numpy qt5-default \ 
    && cd /root/atom_ws/src  && git clone https://github.com/miguelriemoliveira/rviz \ 
    && source /opt/ros/noetic/setup.bash && cd /root/atom_ws && catkin_make  \ 
    && echo "source /root/atom_ws/devel/setup.bash --extend" >> ~/.bashrc \ 
    && cd /root/atom_ws/src/atom/atom_core \ 
    && pip3 install -e .

RUN cp /root/jango_ws/src/tor42_ridgeback/tor42_ridgeback_description/urdf/jango_new.urdf  /tmp/description.urdf
RUN pip3 install numpy==1.21
RUN pip3 install opencv-python==4.9.0.80 opencv-contrib-python==4.9.0.80
RUN apt-get install -y ros-noetic-rviz-visual-tools  
RUN chmod 777 -R /root/atom_ws/src && chmod 777 -R /root/jango_ws/src 



## atom dependancies
# RUN pip3 install pynput==1.7.6 rospy-message-converter==0.5.2 readchar==4.0.5 prettytable==3.10.0  \
#     pytictoc==1.5.3 graphviz==0.20.1
# RUN   

# && catkin_init_workspace \
# && git clone https://github.com/lardemua/atom/tree/noetic-devel \
# && https://github.com/miguelriemoliveira/mmtbot \
# && git clone https://github.com/iris-ua/iris_ur10e -b noetic-devel \
# && git clone https://github.com/iris-ua/iris_ur10e_calibration -b noetic-devel\
# && git clone https://bitbucket.org/DataspeedInc/velodyne_simulator -b master\
# && git clone -b calibration_devel https://github.com/lardemua/universal_robot.git \
# && rm -r iris_ur10e/ur_kinematics 

# RUN cd /root/django_ws && catkin_make_isolated 

SHELL ["conda", "run", "-n", "myenv", "/bin/bash", "-c"]


# && rosdep install --from-paths src -i --rosdistro ${ROS_DISTRO} \