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

This package depends on the ARIAC 2019 competition environment setup located [here](https://bitbucket.org/osrf/ariac/wiki/2019/Home). Please create a similar workspace directory with the packages relevant packages and tools installed.

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

### Installing the package:

Open a terminal in the `ariac_ws/src` directory (or the work space setup for the 2019 competition). Then perform the following commands:

```
git clone git@github.com:cwru-courses/csds373-f21-t2-ariac.git
```

Then, all the packages can be built after navigating to `ariac_ws` and making:

```
catkin_make
```

Then shell can be made aware of the built packages:

```
source devel/setup.bash
```

## Running:
Once all of the packages and files have been installed as specified above and set up, the competition environemtn and order information can be began. The following will run the package to start the environment:

``` 
roslaunch ecse_373_ariac ecse_373_ariac.launch
```

After gazebo window opens and there was time for everything to load, we can then open another terminal under `ariac_ws` and type source the devel (to make it aware of the packages):

```
source devel/setup.bash
```

And finally, the order management and position locator can be ran:

```
rosrun ariac_comp ariac_comp_node
```

## Important Design Decisions:
- The location where orders are taken in (put into a vector) and when they are processed differ. As a result, some differences may be noticeable (even though spinOnce() should force synchronicity).
- Output: Some sample output has been given where:
    - The first info indicates Trigger status
    - Order _ received is the callback functions to load orders.
    - And everything within the dashes (-----) indicates a single order being processed. For the purposes of this lab, this will just be the first order's first shipment's first product's first location.
``` 
kxz167@KRIS-SERV-SLAVE2:~/Documents/arriac_ws$ rosrun ariac_comp ariac_comp_node 
[ INFO] [...]: Start Success: competition started successfully!
[ INFO] [...]: Order 1 Received.
[ INFO] [...]: ------------------------
[ INFO] [...]: Processing Order Begin: 
[ INFO] [...]: Product Type : piston_rod_part
[ INFO] [...]: Located In   : bin4
[ WARN] [...]: Product Position:
[ WARN] [...]: xyz = (0.547798, 0.156758, 0.018934)
[ WARN] [...]: wxyz = (0.621678, -0.329199, -0.626213, 0.336158)
[ INFO] [...]: Processing Order End:   
[ INFO] [...]: ------------------------
```

*NOTES:* 
- While this DOES output the first order, given that orders will continue to flow in, if left open, more subsequent orders will arrive and be processed.
