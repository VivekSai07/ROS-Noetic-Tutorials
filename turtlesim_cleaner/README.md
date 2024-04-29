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
## B) Create Node & make it executable
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
## A) Create Node & make it executable
```bash
cd ~/catkin_ws/src/turtlesim_cleaner/scripts
touch rotate.py
chmod +x rotate.py
```
## Code Explanation
1. First we need to import the packages used on our script.The rospy library is the ros python library, it contains the basic functions, like creating a node, getting time and creating a publisher.The geometry_msgs contains the variable type Twist that will be used, and we define a constant PI that will be required:
```bash
import rospy
from geometry_msgs.msg import Twist
import math
PI = math.pi
```
2. Next we declare our function, initiate our node, our publisher and create the Twist variable.
```bash
def rotate():
    #Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
```
3. Convert the input from degrees to radians.
```bash
angular_speed = speed*2*PI/360
relative_angle = angle*2*PI/360
```
4. Depending on the user's input we decide if the movement will be clockwise or counter-clockwise:
```bash
if clockwise:
    vel_msg.angular.z = -abs(angular_speed)
else:
    vel_msg.angular.z = abs(angular_speed)
```
5. Now , with the rospy.Time.now().to_sec(). we get the starting time t0, and the time t1 to calculate the angular distance and while the actual distance is less than the user's input, it will keep publishing. After we get to the specified angle, we order our robot to stop: 
```bash
while not rospy.is_shutdown():    
    t0 = rospy.Time.now().to_sec()
    curr_angle = 0
    
    while curr_angle < relative_angle:
        vel_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        curr_angle = angular_speed * (t1 - t0)
    
    vel_msg.angular.z = 0
    vel_publisher.publish(vel_msg)
    rotate()
```
6.  The following statement guarantees that if we press ctrl+c our code will stop:
```bash
rospy.spin()
```
