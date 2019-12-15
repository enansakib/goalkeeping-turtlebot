#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

# global variable defined to keep track of the speed of the robot
# so that we can stop the robot smoothly instead of sudden hault
Speed = 0
def callback(dt):
    global Speed
    if dt.data <= 600: # Since the image is resized to 600 width
        if dt.data < 303 and dt.data > 297: # ball is in the middle
            # If the current speed is high then decrease the speed gradually
            # else stop immediately.
            if abs(Speed) > 0.05:
                Speed *= 0.95
            else:
                Speed = 0
            # modify the linear velocity of Twist message
            mv.linear.x = Speed
        else: # ball is not in the middle
            # increase the speed linearly with ball's location, i.e., if the
            # ball is moving far away from the robot speed is higher and if it's
            # moving closer to the robot increase the speed slowly
            Speed = -((dt.data - 300) / 300) * 0.9 # 300 is the middle
            mv.linear.x = Speed
    else: # no ball detected (700 case: see tracking.py), stop the robot gradually
        if abs(Speed) > 0.05:
            Speed *= 0.95
        else:
            Speed = 0
        mv.linear.x = Speed

    pub.publish(mv)

rospy.init_node('move_robot')
mv = Twist()
pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
rospy.Subscriber('/coordinates', Float32, callback)
# Loops infinitely until someone stops the program execution
rospy.spin()
