#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32, Twist

class Bug2:
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

        #Important points for Bug2
        self.start_point = Point32()
        self.start_point.x = 0.0
        self.start_point.y = 0.0

        self.end_point = Point32()
        self.end_point.x = 0.0
        self.end_point.y = 0.0

        self.hit_point = Point32()
        self.hit_point.x = 0.0
        self.hit_point.y = 0.0

        self.hitpoint_to_goal = 0.0

        self.leave_point = Point32()
        self.leave_point.x = 0.0
        self.leave_point.y = 0.0

        self.goal_line_m = 0.0
        self.goal_line_b = 0.0
        self.goal_line_c = 0.0

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
        self.wall_distance_thresh = 0.5
        self.crash_distance = 0.2
        self.orientation_thresh = 0.1
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
        self.end_point.x = data.x
        self.end_point.y = data.y
        self.bug2_init()
    
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
                self.follow_wall()

            self.rate.sleep()

    def bug2_init(self):
        m = 0.0
        b = 0.0
        c = 0.0
        self.start_point.x = self.pose.x
        self.start_point.y = self.pose.y

        if((self.end_point.x - self.start_point.x) != 0):
            m = ((self.end_point.y - self.start_point.y)/
                            (self.end_point.x - self.start_point.x))
            b = self.start_point.y - (m * self.start_point.x)
        else:
            c = self.start_point.x
        self.goal_line_m = m
        self.goal_line_b = b
        self.goal_line_c = c

        self.state = 1

    def go_to_goal(self):
        print("Go to goal")
        d = self.wall_distance_thresh
        if(self.leftfront_dist < d or self.front_dist < d or self.rightfront_dist < d):
            #Register hit point
            self.hit_point.x = self.pose.x
            self.hit_point.y = self.pose.y

            #Register distance from hit point to goal
            self.hitpoint_to_goal = (np.sqrt((np.power(self.end_point.x - self.hit_point.x, 2)) +
                                             (np.power(self.end_point.y - self.hit_point.y, 2))))
            
            #Stop and turn left
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = self.angular_speed_fast

            #Change the state to follow wall
            self.state = 2
        
        elif(not self.angleControl()):
            self.positionControl()

        self.cmdVelPub.publish(self.velocity)

    def on_the_line(self, m, b, c, current_point):
        on_line = False
        #Calculate closest point to goal line
        if(c == 0.0 ):
            goal_line_x = current_point.x
            goal_line_y = (m * goal_line_x) + b

            distance_to_line = goal_line_y - current_point.y

        else:
            distance_to_line = c - current_point.x

        print("distance to line")
        print(distance_to_line)

        if(np.abs(distance_to_line) < self.distance_to_line_thresh):
            leave_point_x = current_point.x
            leave_point_y = current_point.y

            leave_to_goal = np.sqrt(np.power(self.end_point.x - leave_point_x, 2) + 
                                    np.power(self.end_point.y - leave_point_y, 2))
            
            print("Leave to goal")
            print(leave_to_goal)
            print(self.hitpoint_to_goal)
            
            if(leave_to_goal < self.hitpoint_to_goal):
                print("Leave line")
                on_line = True

        return on_line
    
    def follow_wall(self):
        linear_vel = 0.0
        angular_vel = 0.0
        
        if(self.on_the_line(self.goal_line_m, self.goal_line_b, self.goal_line_c, self.pose)):
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = self.angular_speed_fast
            self.cmdVelPub.publish(self.velocity)
            self.state = 1

        else:
            d = self.wall_distance_thresh

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

    
    # TO DO: Implement wrap to pi
    def angleControl(self):
        kpw = 2.0

        #Calcula el error
        desired_angle = np.arctan2(self.end_point.y - self.pose.y, self.end_point.x - self.pose.x)
        # print("desired angle")
        # print(desired_angle)
        desired_angle = desired_angle/(np.pi)
        error = desired_angle - self.orientation
        # print("normalized desired angle")
        # print(desired_angle)
        # print("orientation")
        # print(self.orientation)
        # print("error")
        # print(error)

        if(np.abs(error) > self.orientation_thresh):
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
        error_x = self.end_point.x - self.pose.x
        error_y = self.end_point.y - self.pose.y
        error_l = np.sqrt(error_x ** 2 + error_y ** 2)

        if(np.abs(error_l) > self.pose_thresh): #Goal not reached
            control_output_position = kpl * error_l
            if(control_output_position > self.linear_speed_fast):
                control_output_position = self.linear_speed_fast

            self.velocity.linear.x = control_output_position
        else: #Goal reached
            self.velocity.linear.x = 0.0
            #Go to stopped state
            print("Goal Reached")
            self.state = 0


    


if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("Bug2")
    bug_2 = Bug2()
    print("Navigation Mode Bug 2")
    bug_2.main()