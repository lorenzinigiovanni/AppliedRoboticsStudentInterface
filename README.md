# Robot Planning and Automated Planning Joint Project: Pursuer-Escaper Scenario

A pursuer evader scenario with borders and obstacles inthe map is a good case study to apply motion planning andautomated planning to a real problem. Here, the pursuer has to catch the evader before it reaches one of the gates. To do so, the planning node of a ROS environment is coded. The information obtained by the processing of the map is exploited to encode the problem structure into a graph and then  to  encode  the  problem  in  PDDL. Metric  FF  is  used to obtain the path which will then be refined by an iterative dynamic programming solution of the Dubins problem. The system is able to handle increasing evader behavioural complexities by planning a "fake" path of the evader and encode the information in the pursuer planner.

# Acknowledgment

This is a project for the courses: **Robot Planning** and **Automated Planning** of the University of Trento.

Professors:
- Luigi Palopoli
- Marco Roveri

Students:
- Giovanni Lorenzini
- Simone Luchetta
- Diego Planchenstainer

Used libraries and softwares:
- [AlexRookie/AppliedRoboticsEnvironment](https://github.com/AlexRookie/AppliedRoboticsEnvironment)
- [AlexRookie/AppliedRoboticsStudentInterface](https://github.com/AlexRookie/AppliedRoboticsStudentInterface)
- [Boost](https://www.boost.org)
- [CDT](https://github.com/artem-ogre/CDT)
- [Clipper](http://www.angusj.com/delphi/clipper.php)
- [Metric FF](https://fai.cs.uni-saarland.de/hoffmann/metric-ff.html)

# Installation

Use Ubuntu 16.04 LTS.

## Install the ROS

Setup:

```shell
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full
```

Environment setup:

```shell
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

Dependencies for building packages:

```shell
$ sudo apt install python-rosdep
$ sudo apt install ros-kinetic-rqt-multiplot ros-kinetic-usb-cam
$ sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
$ sudo apt-get install python-catkin-tools
$ sudo apt install chrpath
$ sudo apt-get install libignition-math2-dev
$ sudo apt install ros-kinetic-jsk-visualization
```

Initialize rosdep:

```shell
$ sudo rosdep init
$ rosdep update
```

## Install Gazebo

Setup:

```shell
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Install Gazebo:

```shell
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

## Install OpenCV 3.3.1

Setup:

```shell
$ sudo apt-get update
```

Install dependencies:

```shell
$ sudo apt-get install cmake git libgtk2.0-dev pkg-config
$ sudo apt-get install -y qt5-default libvtk6-dev
$ sudo apt-get install -y zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libjasper-dev libopenexr-dev libgdal-dev
$ sudo apt-get install -y libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev yasm libopencore-amrnb-dev libopencore-amrwb-dev libv4l-dev libxine2-dev
```

Install the library:

```shell
$ wget https://github.com/opencv/opencv/archive/3.3.1.zip
$ unzip 3.3.1.zip
$ rm 3.3.1.zip
$ mv opencv-3.3.1 OpenCV
$ cd OpenCV
$ mkdir build
$ cd build
$ cmake -DWITH_QT=ON -DWITH_OPENGL=ON -DBUILD_EXAMPLES=ON ..
$ make -j4
```

Install OpenCV:

```shell
$ sudo make install
$ sudo ldconfig
```

## Clone the simulator

Clone git:

```shell
$ mkdir ~/workspace
$ cd ~/workspace
$ git clone https://github.com/lorenzinigiovanni/AppliedRoboticsEnvironment simulator
```

Compile the simulator:

```shell
$ cd ~/workspace/simulator
$ catkin build
source ./environment.sh
```

## Clone the project

Clone git:

```shell
$ cd ~/workspace
$ git clone https://github.com/lorenzinigiovanni/AppliedRoboticsStudentInterface project
```

Create build folder:

```shell
$ cd ~/workspace/project
$ mkdir build
```

## Automatic source of the environment

Do this only if the above procedure worked without errors:

```shell
$ echo "source ${AR_CATKIN_ROOT}/environment.sh" >> ~/.bashrc
$ echo "source ${AR_ROOT}/environment.sh > /dev/null 2>&1" >> ~/.bashrc
```

## Download Boost C++ library

```shell
$ cd ~/workspace/project/src
$ wget https://boostorg.jfrog.io/artifactory/main/release/1.77.0/source/boost_1_77_0.tar.gz
$ tar -xf boost_1_77_0.tar.gz
$ rm boost_1_77_0.tar.gz
$ mv boost_1_77_0 boost
```

## Download and compile Metric FF

Download Metric FF:

```shell
$ cd ~
$ wget https://fai.cs.uni-saarland.de/hoffmann/ff/Metric-FF-v2.1.tgz
$ tar zxvf Metric-FF-v2.1.tgz
$ rm Metric-FF-v2.1.tgz
$ mv Metric-FF-v2.1 MetricFF
```

Compile Metric FF and copy it in the project folder:

```shell
$ sudo apt-get install build-essential flex bison
$ cd ~/MetricFF
$ make
$ cp ff ~/workspace/project/src/pddl
```

# Preparation

Create a directory to store debug images (clean the content before each run):

```shell
$ mkdir ~/workspace/images
```

Create a directory to store the robots status (clean the content before each run):

```shell
$ mkdir ~/workspace/state
```

In the file `settings.hpp`, it is possible to set the following parameters:

- Behavioral complexity of the robot, value between 1 and 3;
- If to save plan image in `workspace/images/` folder;
- Length of path the robot takes for a step;
- Offset value to be applied to borders and obstacles;
- Curvature parameter for the robot;
- The absolute path of the workspace folder used for the project.

```shell
$ nano ~/workspace/project/src/settings.hpp
```

Compile:

```shell
$ cd ~/workspace/project/build
$ cmake ..
$ make
$ source ../environment.sh
```

# Execution

Open three terminals.

In the first one run the simulator:

```shell
$ AR_simulator_gui 
```

In the second one run the pipeline:

```shell
$ AR_pipeline
```

In the third one make the robots move, repeat the command until the robots reach the goal:

```shell
$ AR_run
```
