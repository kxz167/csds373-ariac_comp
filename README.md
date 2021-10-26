# CSDS 373 Lab 5: Ariac Comp - Order processing

## Completed by Team 2: 

Kris Zhao (kxz167)\
Sanhita Kumari (sxk1409)\
Michael Koltisko (mek169)

## Important information:

### Development:

Developement was done in Ros Melodic for Ubuntu 18.04 installed natively on hardware.

## Ros Package:

### Name:

```
ariac_comp
```

### Dependencies:

This package depends on the ARIAC 2019 competition environment setup located [here](https://bitbucket.org/osrf/ariac/wiki/2019/Home). Please create a similar workspace directory with the packages.

### Package directory structure:
- ariac_comp (under src)
    - include
        - ariac_comp
    - launch (new launch files)
        - ariac_comp.launch
    - src
        - ariac_comp_node.cpp
    - CMakeLists.txt
    - package.xml
    - README.md

## Installation:

### Ros installation:

Make sure that ros melodic was installed into /opt following the installation procedure listed in the [wikis](http://wiki.ros.org/melodic/Installation/Ubuntu).

Then proceed to source ROS so that your shell is ROS aware:

```
source /opt/ros/melodic/setup.bash
```

### Package pre-requirements:


#### ariac_comp

Open a terminal in the `arriac_ws/src` directory. Then perform the following commands:

```
git clone git@github.com:cwru-courses/csds373-f21-t2-ariac.git
```

### Installing the package:

After the dependencies can be loaded, the package can also be cloned into the `arriac_ws/src`:

```
git clone git@github.com:cwru-courses/csds373-f21-t2-ariac.git
```

Then, all the packages can be built after navigating to `ariac_ws`:

```
catkin_make
```

And the shell can be made aware of the built packages:

```
source devel/setup.bash
```

## Running:
Once all of the packages and files have been installed as specified above and set up, the order information can be outputted. The following will run the package.

``` 
roslaunch ecse_373_ariac ecse_373_ariac.launch
```
After gazebo window opens, open another terminal under araiac_ws and type

```
rosrun ariac_comp ariac_comp_node
```

*NOTES:* 
- The position is outputted as a warning since Dr.Lee proposed to out the location as a ROS_WARN
