# arm_control

## Overview

The arm_control package provides action servers to control TIAGo's arms to pick and place objects.

## Installation

#### Dependencies

The arm_control package has no special dependencies.

#### Building

To build from source, compile the package using:

```bash
catkin build arm_control
```

## Usage

The pick and place servers are launched using:

```bash
roslaunch arm_control arm_control.launch
```

## Launch files

* **arm_control.launch:**  Launches the pick and place action servers to control TIAGo's arms.

## Nodes

### pick_server

Pick action server to control TIAGo's arms for pick actions.

#### Subscribed Topics

* **`/aruco_marker_publisher/markers`** ([aruco_msgs/MarkerArray])

    Array of detected aruco marker poses.

#### Published Topics

* **`/visualization_marker`** ([visualization_msgs/Marker])

    Visualises the pose which TIAGo will move its gripper to to pick an object.

#### Actions

* **`/controls/pick_server`** ([controls_msgs/PickAction])

    Pick action for object to pick. If `use_aruco` is `true`, `pose` can be left empty. When not using aruco markers to find the objects location, set `use_aruco` to `false` and set the pose the object to pick in world frame.


### place_server

Place action server to control TIAGo's arms for place actions.

#### Subscribed Topics

* **`/aruco_marker_publisher/markers`** ([aruco_msgs/MarkerArray])

    Array of detected aruco marker poses.

#### Published Topics

* **`/visualization_marker`** ([visualization_msgs/Marker])

    Visualises the pose which TIAGo will move its gripper to to place an object.

#### Actions

* **`/controls/place_server`** ([controls_msgs/PickAction])

    Place action for object to place. If `use_aruco` is `true`, `pose` can be left empty. When not using aruco markers to find the place location, set `use_aruco` to `false` and set the pose of the place position in world frame. 