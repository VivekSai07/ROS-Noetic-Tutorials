# Move.py
## A) Creating package
- create a new package
```bash
cd ~/catkin_ws/src
catkin_create_pkg turtlesim_cleaner geometry_msgs rospy
```
- build workspace
```bash
cd ~/catkin_ws
catkin_make
```
## B) Create Node
- creating move.py
```bash
cd ~/catkin_ws/src/turtlesim_cleaner/
mkdir scripts
touch move.py
chmod +x move.py
```
## Code Explanation
1. First we need to import the packages used on our script.The rospy library is the ros python library, it contains the basic functions, like creating a node, getting time and creating a publisher.The geometry_msgs contains the variable type Twist that will be used
```bash
import rospy
from geometry_msgs.msg import Twist
```
2. Next we declare the function _**move()**_, initiate our node, our publisher and create the Twist variable.
3. User Input: Prompts the user to input the speed, distance, and direction of movement.
    - **speed** is the velocity at which the robot will move.
    - **distance** is the distance the robot will move.
    - **isForward** is a boolean indicating whether the robot should move forward or backward.
4. Setting Velocity: Based on the user input, it sets the linear velocity of the robot in the x-direction (vel_msg.linear.x). If isForward is True, it sets the velocity to the absolute value of speed, otherwise, it sets it to the negative of speed.
5. Movement Loop: It enters a loop that continues until the distance moved by the robot reaches the specified distance. Inside the loop:
    - It publishes the velocity message.
    - It calculates the current distance moved by the robot by measuring the time difference between the current time and the initial time (t1 - t0).
6. Stopping the Robot:
    - After the loop completes (i.e., when the robot has moved the specified distance), it sets the linear velocity to zero to stop the robot.
    - It publishes the zero velocity message to ensure the robot stops moving.

# Rotate.py

