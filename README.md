# CSDS 373 Lab 4: Ros Bags

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
team2_ros_bags
```

### Dependencies:

This package depends on two other packages that should be installed into the same workspace (cloned into `catkin_ws/src`).

- `navvis_descriptions` by Kris Zhao (kxz167)
    - Found at: `https://github.com/cwru-courses/csds373_f21_kxz167_l2_navvis_descriptions`
- `glennan_maps`
    - Found at: `https://github.com/cwru-eecs-373/maps_glennan`

### Package directory structure:
- team2_ros_bags (from github)
    - config (rviz configuration)
        - laser_topic_config.rviz
    - launch (new launch files)
        - navvis_descriptions_link.launch
        - ros_bags.launch
    - urdf (modified robot description)
        - navvis_imu.xacro
    - CMakeLists.txt
    - package.xml
    - README.md

### Important Notes:
- Launch files:
    - While the assignment called for modifications to the previous lab package, Dr Lee has mentioned that making a copy of the primary launch file and making the edits within this package was satisfactory. `navvis_descriptions_link.launch` represents the modified launch file from previous laboratories and provides a modular link into past packages without modifying them.
- RVIZ Visualization:
    - The robot LaserScan displays have been modified to have a `decay time` of 1 second. I found this provides the best representation of the environment around the robot.
    - There are a couple pre-specified views that have been provided to aid in visualizing the 3D enviroment around the robot.
- Static Transform Publisher:
    - No static transforms were given in the lab. As a result, all static transforms used to circumvent the Robot State Publisher in the launch file were taken first from the URDF / XACRO files, then the RVIZ link viewer running the command `roslaunch navvis_descriptions navvis_Descriptions.launch use_xacro:=true`. A note about links is that importing the XACRO file for the horizontal laser scanner creates geometry transformed with respect to another object (which is why the final location in RVIZ was necessary to use).

## Installation:

### Ros installation:

Make sure that ros melodic was installed into /opt following the installation procedure listed in the [wikis](http://wiki.ros.org/melodic/Installation/Ubuntu).

Then proceed to source ROS so that your shell is ROS aware:

```
source /opt/ros/melodic/setup.bash
```

### Package pre-requirements:
Before installing the team2_ros_bags package, prepare the workspace with the two other required components.

#### glennan_maps

Open a terminal in the `catkin_ws/src` directory. Then perform the following commands:

```
git clone git@github.com:cwru-eecs-373/maps_glennan.git
```

#### navvis_descriptions

Open a terminal in the `catkin_ws/src` directory. Then perform the following commands:

```
git clone git@github.com:cwru-courses/csds373_f21_kxz167_l2_navvis_descriptions.git
```

### Installing the package:

After the dependencies can be loaded, the package can also be cloned into the `catkin_ws/src`:

```
git clone git@github.com:cwru-courses/team2_ros_bags.git
```

Then, all the packages can be built after navigatin to `catkin_ws`:

```
catkin_make
```

And the shell can be made aware of the built packages:

```
source devel/setup.bash
```

### Loading the bags:

As all bag files have been ommitted from the github repository and package, they need to be downloaded and located on the client machines:

```
https://drive.google.com/drive/u/1/folders/1ptmPrQc8g12GmbfNCklQ2fOF5rFum46-
```

## Running:
Once all of the packages and files have been installed as specified above and set up, the visualizations can be launched. The following will run the package with alternative possibilities provided through `options`:

``` 
roslaunch team2_ros_bags ros_bags.launch [options]
```
Where options are of the following form:
- launch_bags:=[true/false(default)]
    - Launch the bag playback with the following options:
    - use_gui:=[true/false(default)]
        - Utilize `rqt_bag` to replay bags or the command line tool.
    - bag_file_path:=<file_path> (No default provied)
        - Full qualified path to the bag files. REQUIRED if launching ros_bag functionality.
        - No default is provided because there is no information on where the bag files will be located.
- launch_map:=[true/false(default)]
    - Launches the provided Glennan map to attach into RVIZ
- use_sim time:=[true(default)/false]
    - Whether or not to depend on a simulated clock output by bag playback.
- rviz_config:=<config_type> (default = full.rviz)
    - Three options have been provided to view the status as defined in the deliverables but we recommend always using full and toggling various options:
    1. full.rviz - The final view in RVIZ with the views, map, and laserscans.
    2. laser_only.rviz - The view with laserscans added on top of the modified robot.
    3. original.rviz - The rviz configuration that was pulled from the previous lab showing only the updated robot.

So in order to run the program launching both bag playback, and the map server, would require:
```
roslaunch team2_ros_bags ros_bags.launch launch_bags:=true launch_map:=true bag_file_path:=<path_to_bag_files>.bag
```

*NOTES:* 
- The included bags playback will launch dependent on a simulated clock (`--clock` parameter set) and is meant to be a QoL option providing one launchpoint for full functionality.
- The only interface to the package should be through `ros_bags.launch`. Users are not intended to use the `navvis_descriptions_link.launch`. All configurations can be speicifed thorugh the ros_bags launch file (such as only showing RVIZ, and running (or not) the map and bag servers).
