## Database package


## Overview

The database package will startup the database and make sure that it can be checked and updated. The database will consist of all the products available in the supermarket. Next to all the available products it will also contain the locations, aruco id and amount of all the products available.

For example for the product "milk" it will contain the information:
```bash
milk:
  pose:
    position:
      x: -0.5
      y: -1.35
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.707
      w: 0.707
  amount: 2
  aruco_id: 33
  use_aruco: true
```

Furthermore the database will contain the location of the basket and the coordinates of the home base in two separate yaml files.


## Installation

#### Dependencies

The database package has no special dependencies.

#### Building

To build from source, compile the package using:

```bash
catkin build database
```

## Usage

The database node can be launched through:

```bash
roslaunch database database.launch
```

## Launch files

* **database.launch:**  Launches the database so that it will be accessible for checking and updating.

## Nodes

### database.py

This node will subscribe to two servers namely UpdateDatabase.srv and CheckDatabase.srv
It will respond with a message, the type of the message can be found in custom_msgs UpdateDatabase.srv and CheckDatabase.srv  

## YAML files

### database.yaml

In this file all the products of the database can be found. Other information of the product can also be found in this file like the pose position and orientation, amount, aruco_id, use_aruco and the gripper side.  

### basket.yaml

This file containts information which is necessary to pick up the basket at the beginning of each order. The basket will stand at the same location every order.

### home.yaml

This file contains the exact coordinates of where TIAGO needs to be to be able to pick up and drop the basket.
