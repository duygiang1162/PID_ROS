#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64


class Exercise1():
     def __init__(self):
        
        self.x_publisher = rospy.Publisher('/planar_2dof/joint1_position_controller/command', Float64, queue_size=1)
        self.y_publisher = rospy.Publisher('/planar_2dof/joint2_position_controller/command', Float64, queue_size=1)
        self.x = Float64
        self.y = Float64
        self.state = 0
        self.rate = rospy.Rate(10) # 10hz
        self.ctrl_c = False

        rospy.on_shutdown(self.shutdownhook)   

     def cartesian_xy(self):
         while not self.ctrl_c:

             if self.state == 0:

                self.theta1 =0.785398
                self.theta2 =0.436332
                self.state = 1
                rospy.sleep(5)

             elif self.state == 1:

                self.theta1 =0.9
                self.theta2 =0.49
                self.state = 2
                rospy.sleep(5)
             elif self.state == 2:

                self.theta1 =0.872665
                self.theta2 =0.471239
                self.state = 3
                rospy.sleep(5)

             else:
                 self.state = 4

             
             self.x_publisher.publish(self.x)
             self.x_publisher.publish(self.y)
             self.rate.sleep()

    
         

     def shutdownhook(self):
        # trabaja mucho mejor que el rospy.is_shutdown()
        self.ctrl_c = True


if __name__ == '__main__':
    rospy.init_node('publish_test', anonymous=True)
    exercise_object = Exercise1()
    try:
        exercise_object.cartesian_xy()
    except rospy.ROSInterruptException:
        pass