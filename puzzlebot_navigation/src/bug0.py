#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32, Twist

class Bug0:
    def __init__(self):
        #LaserScan readings
        self.walls = [9999.9] * 360
        self.left_dist = 9999.9
        self.leftfront_dist = 9999.9
        self.front_dist = 9999.9
        self.rightfront_dist = 9999.9
        self.right_dist = 9999.9

        #Current position and orientation
        self.pose = Point32()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.orientation = 0.0

        #State variable (0 = go to goal, 1 = follow wall)
        self.state = 0

        #Important points for Bug0
        self.start_point = Point32()
        self.start_point.x = 0.0
        self.start_point.y = 0.0

        self.set_point = Point32()
        self.set_point.x = 0.0
        self.set_point.y = 0.0

        self.angle_to_goal = 0

        #Velocity vector
        self.velocity = Twist()
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0

        #Max speeds
        self.linear_speed_fast = 0.6
        self.linear_speed_slow = 0.3
        self.angular_speed_slow = 0.6
        self.angular_speed_fast = 1.25

        #Tolerances
        self.wall_distance_thresh = 0.6
        self.crash_distance = 0.2
        self.orientation_thresh = 0.2
        self.pose_thresh = 0.2
        self.distance_to_line_thresh = 0.2

        self.alternate = 1.0

        self.rate = rospy.Rate(10)
        rospy.Subscriber("/puzzlebot_1/scan", LaserScan, self.laserCallback)
        rospy.Subscriber("/puzzlebot_1/base_controller/odom", Odometry, self.poseCallback)
        rospy.Subscriber("/setpoint", Point32, self.endPointCallback)
        #Setup de publishers
        self.cmdVelPub = rospy.Publisher("/puzzlebot_1/base_controller/cmd_vel", Twist, queue_size=1)

    def laserCallback(self, data):
        self.walls = np.clip(data.ranges, 0, 10)
        self.left_dist = self.walls[269]
        self.leftfront_dist = self.walls[224]
        self.front_dist = self.walls[179]
        self.rightfront_dist = self.walls[134]
        self.right_dist = self.walls[89]
        # print("Received")
        # print(self.walls)

    def poseCallback(self, data):
        self.pose.x = data.pose.pose.position.x
        self.pose.y = data.pose.pose.position.y
        self.orientation = data.pose.pose.orientation.z
        # print(self.pose)

    def endPointCallback(self, data):
        self.set_point.x = data.x
        self.set_point.y = data.y
        self.state = 1
    
    def main(self):
        while not rospy.is_shutdown():
            #State 0: Stopped
            if(self.state == 0):
                self.velocity.linear.x = 0.0
                self.velocity.angular.z = 0.0
                self.cmdVelPub.publish(self.velocity)

            #State 1: Go directly to goal
            elif(self.state == 1):
                self.go_to_goal()

            #State 2: Follow the wall
            elif(self.state == 2):
                if(self.leave_wall(self.set_point, self.pose)):
                    self.state = 1
                else:
                    self.follow_wall()

            self.rate.sleep()


    def go_to_goal(self):
        print("Go to goal")
        #Check for walls on the front
        d = self.wall_distance_thresh
        if(np.nanmin(self.walls[135:225]) < self.wall_distance_thresh):
            #Stop and turn left
            if(np.nanmin(self.walls[135:225]) < self.crash_distance):
                self.velocity.linear.x = -self.linear_speed_fast
            else:
                self.velocity.linear.x = 0.0
                self.velocity.angular.z = self.angular_speed_fast

                #Change the state to follow wall
                self.state = 2
        
        elif(not self.angleControl()):
            self.positionControl()

        self.cmdVelPub.publish(self.velocity)

    def leave_wall(self, set_point, current_point):
        leave_wall = False
        self.check_angle(set_point, current_point)
        d = self.wall_distance_thresh
        a = self.angle_to_goal
        # print("Angle to goal")
        # print(a)

        if(self.walls[a-1] > d):
            print("No obstacle to goal")
            self.velocity.linear.x = self.linear_speed_slow
            self.velocity.angular.z = self.angular_speed_slow
            self.cmdVelPub.publish(self.velocity)
            leave_wall = True

        return leave_wall
    
    def follow_wall(self):
        linear_vel = 0.0
        angular_vel = 0.0
        
        d = self.wall_distance_thresh

        #TO DO: Change wall to follow depending on angle
        #Wall on front, turn left to align
        if(self.front_dist < d):
            if(self.front_dist <= self.crash_distance):
                #Too close, go back
                linear_vel = -self.linear_speed_slow
                print("Too close to wall on front")
            else:
                angular_vel = self.angular_speed_fast
                print("wall on front")
        
        #No wall detected on the right or front, search for wall by turning right and going forward
        elif(self.rightfront_dist > d):
            print("No walls")
            linear_vel = self.linear_speed_slow
            angular_vel = - self.angular_speed_fast
        
        #Wall on the right, go forward
        elif(self.rightfront_dist < d):
            if(self.rightfront_dist <= self.crash_distance):
                #Too close, go back and left
                print("Too close to wall on the right")
                linear_vel = self.linear_speed_slow * self.alternate
                angular_vel = self.angular_speed_fast
                self.alternate = -self.alternate
            else:
                print("Following wall")
                linear_vel = self.linear_speed_fast

        self.velocity.linear.x = linear_vel
        self.velocity.angular.z = angular_vel

        self.cmdVelPub.publish(self.velocity)

    def check_angle(self, set_point, pose):
        #Calcula el error
        desired_angle = np.arctan2(set_point.y - pose.y, set_point.x - pose.x)
        angle_to_goal_deg = desired_angle * 180 / np.pi
        self.angle_to_goal = int(np.rint(angle_to_goal_deg)) + 90
        print("desired angle")
        print(angle_to_goal_deg)
        desired_angle_normalized = desired_angle/(np.pi)
        error = desired_angle_normalized - self.orientation
        # print("normalized desired angle")
        # print(desired_angle)
        # print("orientation")
        # print(self.orientation)
        # print("error")
        # print(error)

        wrong_angle = np.abs(error) > self.orientation_thresh

        return wrong_angle, error
    
    # TO DO: Implement wrap to pi
    def angleControl(self):
        kpw = 2.0

        wrong_angle, error = self.check_angle(self.set_point, self.pose)

        if(wrong_angle):
            control_output_angle = kpw * error
            if (control_output_angle > self.angular_speed_fast):
                control_output_angle = self.angular_speed_fast

            self.velocity.linear.x = 0.0
            self.velocity.angular.z = control_output_angle
            return True
        else:
            self.velocity.angular.z = 0.0
            return False
        
    def positionControl(self):
        kpl = 1.0

        # Calcula el error
        error_x = self.set_point.x - self.pose.x
        error_y = self.set_point.y - self.pose.y
        error_l = np.sqrt(error_x ** 2 + error_y ** 2)

        if(np.abs(error_l) > self.pose_thresh): #Goal not reached
            control_output_position = kpl * error_l
            if(control_output_position > self.linear_speed_fast):
                control_output_position = self.linear_speed_fast

            self.velocity.linear.x = control_output_position
        else: #Goal reached
            self.velocity.linear.x = 0.0
            #Go to stopped state
            print("Goal reached")
            self.state = 0


    


if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("Bug0")
    bug_0 = Bug0()
    print("Navigation Mode Bug 0")
    bug_0.main()