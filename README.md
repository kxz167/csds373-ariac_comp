# CSDS 373: Ariac Comp - Order Filling: Direct IK

## Completed by Team 2: 

Kris Zhao (kxz167)\
Sanhita Kumari (sxk1409)\
Michael Koltisko (mek169)

## Important information:

Phase 3 and Phase 4 were not tagged as we were already in the required sophisticated environment.
Notes:
1) The camera is used as the reference frame for placing the tools on the tray, but it is functional.
2) The program goes on sleep as soon as the arm aborts in order to avoid the abort propagation.

### Block Diagram:

![GitHub Logo](/robo.PNG)

### Development:

Developement was done in Ros Melodic for Ubuntu 18.04 installed natively on hardware.

Development was done with the `ecse_373_ariac` package on commit `a1f1fcfc8551...` published Nov.3. It appears the fix for positive starting position has not been pushed or is not loading correctly. Therefore angles had to be manipulated to remain negative for specifically the arm lift joint. 

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
    - src
        - ariac_comp_node.cpp
    - CMakeLists.txt
    - package.xml
    - README.md
- ecse_373_ariac (cloned package)

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
- Additionally, since there are components within the main order processing loop that requires certain logical camera or joint information (which isn't guarenteed to be populated), we will go ahead and keep looping without processing orders until the corresponding information has been captured.

### Angle filtering / Angle modifications:
All filtering here is meant to select an optimal solution to choose. The criteria are the following:
- If the robot is facing away from the bins, the lift and elbows need to be positioned in an up position, similarly to when the root is facing towards the bins.
- The wrist must be modified so that when it twists out, the angles will not overlap.

### Crucial methods:

There are a couple fundamental methods which were created to accomplish the tasks of the lab. These included:

- optimal_solution_index()
    - This goes ahead and filters the inverse kinematic results based on correct angles.
- patch_trajectory_point()
    - This patches a single trajecory point so that the lift angle is correct
- patch_trajectory()
    - This patches all trajectory points within a given trajectory
- ik_point()
    - This method goes ahead and calculates inverse kinematics and creates a trajectory point directly (which can be added to a trajectory)
- ik_point_2()
    - This method goes ahead and calculates inverse kinematics and creates a trajectory point directly but with a modified filter.

### Helper methods:

Asside from the core functionality of the lab, other methods were used to generalize and standardize various processes. These are outlined below:

- base_trajectory()
    - This method creates a starting trajectory with the proper headers 
- actuator_position()
    - This calculates the linear arm actuator position given the target bin and product on the bin.
- print_pose()
    - This prints out the provided pose with ROS_WARN messages.
- print_trajectory_points()
    - Prints out all of the points within a given trajectory.
- pose_wrt_arm()
    - This transforms poses from certain cameras into the current arm location.
- send_trajectory()
    - Sends a single trajectory to the action server and waits for response.
- agv_filter()
    - Fixes coordinates for agv placement.
- optimal_solution_index()
    - Certain angles are filtered depending on location.

## Auxilery methods:
Our group had substantial issues with bad behavior of the ariac arm1. These included various segfaults, and collisions with bins or cameras. To try and remedy these problems, we implemented various other methods to try and move the arm accordingly.

The fixes provided by Lee does not appear to have propogated to our machines and fixes then had to be added.

- dist()
    - This calculated the shortest distance the robot arm would have to move.
- optimal_solution()
    - Utilizes dist() in order to create the shortest path trajectory between allarms
- linear_move()
    - This calculates n points between the end effector and the part in a straight line. Was created when it appeared the arm was not moving in a straight line.
