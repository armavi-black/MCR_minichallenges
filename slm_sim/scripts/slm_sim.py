#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

#Declare Variables to be used
#Variables físicas dadas por el problema
k = 0.01
m = 0.75
l = 0.36
a = l/2
g = 9.8
tau = 0.0
x1 = 0.0
x2 = 0.0
dt = 1/100

# Setup Variables to be used
joint = JointState()
joint.name = ["joint2"]
joint.position = 0.0
joint.velocity = 0.0

# Declare the input Message

# Declare the  process output message
def init_joints():
    global joint
    joint.header.frame_id = ""
    joint.header.stamp = rospy.Time.now()
    joint.name = ["joint2"]
    joint.position = [0.0]
    joint.velocity = [0.0]
    joint.effort = [0.0]

#Escribe los valores de posición y velocidad a una variable JointState
def jointState(x1, x2):
    global joint
    joint.header.stamp = rospy.Time.now()
    joint.position = [x1]
    joint.velocity = [x2]


#Define the callback functions
def callbackTau(msg):
    global tau
    tau = msg.data

  #wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi


def stop():
    print("stopping sim")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SLM_Sim")

    print("test")

    #Get Parameters   

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))
    rospy.on_shutdown(stop)
    

    # Setup the Subscribers
    rospy.Subscriber("/tau", Float32, callbackTau)

    #Setup de publishers
    pubJoint = rospy.Publisher("/joint_states", JointState, queue_size=10)

    init_joints()

    print("The SLM sim is Running")
    while not rospy.is_shutdown():

        #Modelo dinámico despejado para x1(q) y x2(q punto)
        x1 += x2*dt
        x2_dot = (1/(4/3 * m * a * a)) * (tau - (m*g*a * np.cos(x1)) - k*x2)
        x2 += x2_dot*dt

        #Limita el valor de la posición a una rotación
        x1 = wrap_to_Pi(x1)

        #Establece la entrada de control a 0 (Busca estabilizarse hacia abajo)
        #para que mandarle un valor tau este solo actúe un instante y funcione como un kick
        tau = 0.0

        jointState(x1,x2)

        pubJoint.publish(joint)

        #Wait and repeat
        loop_rate.sleep()