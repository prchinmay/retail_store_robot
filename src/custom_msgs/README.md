# Custom_msgs

## Overview

This package contains the custom services and actions (and messages). These are used by the other packages.

**Keywords:** custom messages, services, actions


## Build

To build this package and be able to use the different service and message files, the package should be build.
Note! This packages is tested on ROS melodic, so no guarantees can be given when the package is in an other version of ROS.

To be able to build this package, make sure you have [ROS melodic installed](http://wiki.ros.org/melodic/Installation) and created an [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
Next clone the repository in your src folder and run the following commands:

```
$ source /opt/ros/melodic/setup.bash
$ catkin build
```

## Usage

The package is used as an dependency of other package, because of this the commands which will be explained below are only to have an idea of the message available in this package. it will also be explained how to add this package as an dependency to your own package.

### Visualize

To check the available srv run the following command:
```
$ rossrv list | grep custom_msgs
```
To check how the srv are build up run the following command:
```
$ rossrv show <custom_msgs/SRV_FILE>
```

By building the package the action file are build to different .msg file, to check which one are available run:
```
rosmsg list | grep custom_msgs
```
To check how the files are build up use:
```
$ rosmsg show <custom_msgs/MSG_FILE>
```


### Dependency

As explained this package can be used as a dependency, in order to do this the following should be added to the package.xml file of your own package:
```
<build_depend>custom_msgs</build_depend>
<exec_depend>custom_msgs</exec_depend>
```
If adding this gives error you could also try to use this instead:
```
<depend>custom_msgs</depend>
```

Next the dependency should be added to your CMakeLists.txt:
```
find_package(catkin REQUIRED COMPONENTS
  custom_msgs
)

generate_messages(
  DEPENDENCIES custom_msgs
)
```
