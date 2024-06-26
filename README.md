# ROS-Noetic-Tutorials

Robot Operating System (ROS) is an open-source framework that facilitates the development of robotics applications by providing tools, libraries, and communication structures for modular and reusable code. It is not an operating system but a set of software frameworks designed for robot software development. Here are some basic definitions in ROS:

- **Node**: A node is an independent process that performs a specific function or task in a ROS system. Nodes can be written in various programming languages, such as C++ or Python, and can run on different devices connected in a network.
- **Topic**: A topic is a communication channel for exchanging messages between nodes. Nodes can publish or subscribe to topics, which are named and can carry different types of data.
- **Message**: A message is a data structure that contains a set of fields, such as integers, floats, or strings. Messages are used to communicate data between nodes through topics.
- **Service**: A service is a way for nodes to call a function or method on another node. Services are defined by a request and response message type and allow for synchronous communication between nodes.
- **Package**: A package is a collection of related nodes, messages, services, and other resources that form a functional unit in ROS. Packages can be shared and reused across different projects and applications.
- **Workspaces**: A workspace is a directory structure that contains the source code and build files for one or more packages. Workspaces are used to manage the dependencies and build process of ROS applications.
- **ROS Master**: The ROS Master is a central hub that keeps track of the nodes, topics, and services in a ROS system. It provides a registry for nodes to find and communicate with each other.
- **Publisher**: A publisher is a ROS node that sends or publishes messages to a specific topic. A topic is a named bus or communication channel in ROS that allows nodes to exchange messages. Publishers advertise their availability to publish messages on a specific topic to the ROS master, which maintains a registry of all active nodes and topics. When a node subscribes to a topic, the master notifies the publisher, which then establishes a direct connection with the subscriber to send messages.
- **Subscriber**: A subscriber is a ROS node that receives or subscribes to messages from a specific topic. Subscribers register their interest in a topic with the ROS master, which then notifies the publisher when a new subscriber connects to the topic. The publisher then sends messages directly to the subscriber, bypassing the master. Subscribers can specify a callback function that gets executed whenever a new message is received on the subscribed topic.

## Creating a workspace for catkin
To create and build a catkin workspace:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
Before continuing, source your new setup.*sh file:
```bash
source devel/setup.bash
```

## Creating a catkin Package
1. Change to the source space directory of the catkin workspace:
```bash
cd ~/catkin_ws/src
```
2. Now, use the catkin_create_pkg script to create a new package called '*my_robot_controller*' which depends on rospy, turtlesim, and geometry_msgs:
```bash
catkin_create_pkg my_robot_controller rospy turtlesim geometry_msgs
```
This will create a *my_robot_controller* folder which contains a package.xml and a CMakeLists.txt, which have been partially filled out with the information you gave catkin_create_pkg.
3. Now we need to build the packages in the catkin workspace:
```bash
cd ~/catkin_ws
catkin_make
```
4. To add the workspace to your ROS environment you need to source the generated setup file:
```bash
. ~/catkin_ws/devel/setup.bash
```
