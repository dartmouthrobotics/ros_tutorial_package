# Tutorial: Example Package

Example of a package for running a ROS nodes.

This is a work in progress. Feel free to ask for more explanations, tutorials, and examples.

NOTE: this example is for python based packages. There is slight differences when writing C++ ROS packages. If you want an example, feel free to reach out.

[Dartmouth Reality and Robotics Lab](http://rlab.cs.dartmouth.edu/home/)

Author: Monika Roznere

## Getting Started

This package has been tested on Ubuntu 16.04 with ROS Kinetic.

Set up your catkin workspace:
```
mkdir -p catkin_ws/src
cd catkin_ws/src
```

## Dependencies

No unique depencencies are required.

However, here one would list the required dependencies for your package. No need to include `rospy` or other simple ROS related packages. Instead list packages or libraries that are typically not installed with Python or ROS.

Examples (NOT REQUIRED HERE):

* openCV
```
pip install opencv-python
```

* [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)
Or other ROS packages required in the `catkin_ws`

* mention that Gazebo, Stage, or another simulator is required

* [matplotlib](https://matplotlib.org/3.2.2/index.html)
```
sudo apt-get build-dep python-matplotlib
```


## Install

Clone the repository. Then build and source:
```
cd catkin_ws/src
git clone https://github.com/dartmouthrobotics/ros_tutorial_package.git
cd ..
catkin_make
source devel/setup.bash
```

Check if the `start_example.py` file in `scripts` folder is executable. Scripts called by launch files must be an executable. All other scripts that are used (imported) by the executable scripts must be in `src\<package_name>` directory. They do not have to be an executable.

To check either:
* use command `ls` and see if the file is displayed with green font
* or use command `ls -l` and check if the file is given executable permissions. There should be 'x' in the left column for file permissions, e.g. -rwxrwxr-x.

If not already an executable, run:
```
chmod +x start_example.py
```

## Run

Start ROS master:
```
roscore
```

Some parameters that can be set:
* `out_file` = \<File to write data too; Default: test.txt\>
* `rate` = \<ROS rate parameter; Default: 10.0\>

Check out the scripts for how they are used.

To set parameters, in a new terminal tab:
```
rosparam set /<parameter_name> <value>
```

Example:
```
rosparam set /rate 5
```

To run a launch file:
```
roslaunch tutorial_package example.launch
```

## Formatting

I like to use [`roslint`](http://wiki.ros.org/roslint) for cleaning the format of my code.

Check `CMakeLists.txt` and `package.xml` to see how to include this dependency.

To run:
```
catkin_make roslint_<package_name>
```

Example:
```
catkin_make roslint_tutorial_package
```


## Extra: Git

Be in the folder of the git repo to execute such commands.

To pull from git (update your version of the repo):
```
git pull
```

To push your version of the repo to git:
```
git add .
git commit -m "in quotes, explain your update"
git push
```
