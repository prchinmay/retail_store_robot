# input_handling

## Overview
This package will contain the HRI part of our project.
The HRI part of our project will be limited because in our team there was no one assigned to this task.


This package consists of four nodes, of which two ask for input of the user.
The input_request.py and continue_request.py will interact with the client through the terminal. The input_handling.py will make sure all the input handling services are running. The update_order.py will update the order of the products so that TIAGo will follow the shortest path.



**Keywords:** input_handling, input_request, continue_request, update_order

## Usage

Note! When running this commands it is assumed that your workspace is build and sourced!

The run the service requests separately run the following commands:

```
$ roscore
```
```
$ rosrun input_handling input_handling.py
```
```
$ rosservice call /input_request/service True
```

```
$ rosservice call /continue_request/service True
```
```
$ rosservice call /update_order/service True
```




## Nodes

This package contains three service nodes which can be called as described in the usage section. The nodes are as following:

* **`/input_request/service`** ([custom_msgs/InputRequest])

	Returns information about the desired list of products. You can trigger the computation from the console with

		rosservice call /input_request/service True

* **`/continue_request/service`** ([custom_msgs/ContinueRequest])

	Returns a boolean value whether or not to continue with the specified list of products. You can trigger the computation from the console with

		rosservice call /continue_request/service True

* **`/update_order/service`** ([custom_msgs/UpdateOrder])

	Returns information about the desired order of the products. You can trigger the computation from the console with

		rosservice call /update_order/service True

The last node will make sure that every service will be running and will be launched from the start.
