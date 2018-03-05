#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
import math
import random

##########################
# BEGIN Global Variable Definitions
front_distance = -1.0
wall_distance = -1.0
# END Global Variable Definitions
##########################

##########################
# BEGIN ROS Topic Callback Function [DON'T MESS WITH THIS STUFF]
##########################
def laserCallback(laser_data):
#This function will set front_distance and wall_distance.  Both of these work in the same way.  If something is closer than the detection_distance for that sensor, the value will tell how close something is.  If nothing is closer than the detection distance, the value of -1 will be used.

    global front_distance
    global wall_distance

    front_detection_distance = 0.9
    wall_detection_distance = 0.9

    wf_min_right = 10.0
    wf_min_front = 10.0

    cur_angle = laser_data.angle_min

    for i in range(len(laser_data.ranges)):
        if abs(cur_angle + math.pi/2) < (math.pi / 8):
            #Wall sensor ((-5/8)*math.pi <= cur_angle <= (-3/8)*math.pi)
            if laser_data.ranges[i] < wf_min_right:
                wf_min_right = laser_data.ranges[i]

        if abs(cur_angle) < (math.pi / 8):
            #Front sensor ((-1/8)*math.pi <= cur_angle <= (1/8)*math.pi)
            if laser_data.ranges[i] < wf_min_front:
                wf_min_front = laser_data.ranges[i]

        cur_angle = cur_angle + laser_data.angle_increment

    #Set the sensor variables
    front_distance = -1
    wall_distance = -1
    if wf_min_front < front_detection_distance:
        front_distance = wf_min_front
    if wf_min_right < wall_detection_distance:
        wall_distance = wf_min_right

############################
##### END CALLBACK FUNCTION
############################

def wall_force(wall_distance):
    if wall_distance == -1:
        return mag_to_xy([0, math.pi])

    else:
        return add_forces(mag_to_xy([get_pf_magnitude_exponential(1-wall_distance), -math.pi/3]), mag_to_xy([get_pf_magnitude_constant(wall_distance), 0]))  # x, y (robot)

def obstacle_force(front_distance):
    if front_distance == -1:
        return [0,0]
    else:
        return mag_to_xy([-get_pf_magnitude_exponential(front_distance), -math.pi/3])  # x, y (robot)

if __name__ == '__main__':
    rospy.init_node('lab3', anonymous=True) #Initialize the ros node
    pub = rospy.Publisher('cmd_vel', Twist) #Create our publisher to send drive commands to the robot
    rospy.Subscriber("base_scan", LaserScan, laserCallback) #Subscribe to the laser scan topic

    rate = rospy.Rate(10) #10 Hz

    #SENSOR VARIABLES
    global front_distance #How close is something in front of the robot (-1 = nothing is close)
    global wall_distance  #How close is something on the right side of the robot (-1 = nothing is close)

    #########################################
    # LAB 3 VARIABLE DECLARATION CODE : BEGIN
    #########################################

    # PART C CODE HERE:
    #  Define and initialize variables that
    #  you need inside the main loop

    wander_angle = random.randint(-5, 5)
    turn_count = 0
    state = 0

    front_turn_threshold = 0.7  # How close, really, do we have to be to an obstacle to turn?
    wall_dist_threshold = 0.48  # how close should we follow the wall?


    #######################################
    # LAB 3 VARIABLE DECLARATION CODE : END
    #######################################

    while not rospy.is_shutdown():

        ###############################
        # LAB 3 INNER LOOP CODE : BEGIN
        ###############################

        twist = Twist()

        # PART C CODE HERE:
        # Make sure that twist gets set with your drive command
        wall_detected = True if (wall_distance != -1) else False
        obstacle_detected = True if (front_distance != -1) else False

        wall_near  = True if (wall_distance != -1  and wall_distance <= wall_dist_threshold) else False
        obstacle_near = True if (front_distance != -1 and front_distance <= front_turn_threshold) else False

        if state == 0:  # WANDER
            twist.linear.x = 1
            if turn_count < 10:
                twist.angular.z = wander_angle
                turn_count += 1
            else:
                twist.angular.z = 0

            if obstacle_detected:
                state = 1
            elif (not obstacle_detected) and wall_detected:
                state = 2
            else:
                state = 0

        elif state == 1:  # TURN LEFT
            turn_count = 0
            twist.angular.z = math.pi/3
            twist.linear.x = 0.1
            if wall_detected and not wall_near and not obstacle_near:
                state = 2
            elif wall_near and not obstacle_near:
                state = 3
            else:
                state = 1

        elif state == 2:  # TURN TO WALL
            twist.angular.z = -math.pi/3
            twist.linear.x = 0.25
            if obstacle_near:
                state = 1
            elif wall_near and not obstacle_near:
                state = 3
            else:
                state = 2

        elif state == 3:  # FOLLOW WALL
            # twist.angular.z = -math.pi/16 # slight curve toward wall to correct drift
            twist.angular.z = 0
            twist.linear.x = 1

            if not wall_near and not obstacle_near:
                state = 2
            elif obstacle_near:
                state = 1
            else:
                state = 3

        print "State: ", state, " wall_distance: %.2f" % wall_distance, " front_distance: %.2f" % front_distance


        #############################
        # LAB 3 INNER LOOP CODE : END
        #############################

        #Publish drive command
        pub.publish(twist)
        rate.sleep() #Sleep until the next time to publish

    twist = Twist()
    pub.publish(twist)
