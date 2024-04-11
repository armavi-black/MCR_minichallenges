#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

# impulse = 35.0
#Strength of the impulse
impulse = rospy.get_param("/impulse", 35.0)

#Duration between impulses
duration = rospy.get_param("/duration", 10.0)



def stop():
    print("stopping sim")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("impulse_generator")

    #Get Parameters   

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))
    rospy.on_shutdown(stop)
    

    # Setup the Subscribers

    #Setup de publishers
    pubImpulse = rospy.Publisher("/tau", Float32, queue_size=10)

    print("The impulse generator is Running")
    while not rospy.is_shutdown():

        pubImpulse.publish(impulse)

        rospy.sleep(0.01)

        pubImpulse.publish(0.0)

        rospy.sleep(duration)

        #Wait and repeat
        loop_rate.sleep()