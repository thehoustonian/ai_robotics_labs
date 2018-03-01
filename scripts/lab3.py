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

def drive_from_force(force):

    #####################################################
    #PARAMETERS : MODIFY TO GET ROBOT TO MOVE EFFECTIVELY

    #This is multiplied by the angle of the drive force to get the turn command
    turn_multiplier = 0.90

    #If the absolute value of the angle of the force direction is greater than this, we only spin
    spin_threshold = math.pi/3
    # spin_threshold = math.pi/10  # extra credit pt 4

    #This is multiplied by the magnitude of the force vector to get the drive forward command
    drive_multiplier = 0.75

    #END OF PARAMETERS
    #####################################################

    #The twist command to fill out and return
    twist = Twist()

    #Determine the angle and magnitude of the force
    force_angle = math.atan2(force[1],force[0])
    force_mag = math.hypot(force[0],force[1])

    #Get turn speed
    twist.angular.z = turn_multiplier * force_angle

    #Do we just spin?  Only drive forward (twist.linear.x) if angle is small
    if abs(force_angle) < spin_threshold:
        twist.linear.x = drive_multiplier * force_mag

    return twist

def get_pf_magnitude_linear(distance):
    max_strength = 5
    strength = distance * max_strength

    return strength #CHANGE TO RETURN THE VALUE YOU COMPUTE

def get_pf_magnitude_exponential(distance):
    distance_mult = 1
    distance *= distance_mult
    return 1/(distance * distance)

def get_pf_magnitude_constant(distance):
    max_strength = 1.0

    return max_strength

def add_forces(force1, force2):
    assert len(force1) == len(force2), "Force vectors differ in length"
    return [force1[0] + force2[0], force1[1] + force2[1]]  # x, y

def wrap_angle(angle):
    while angle >= math.pi:
        angle = angle - 2*math.pi

    while angle <= -math.pi:
        angle += 2*math.pi

    return angle

def wall_force(wall_distance):
    if wall_distance == -1:
        return [0, 0]
    elif wall_distance > 0.3:
        return [get_pf_magnitude_exponential(1-wall_distance), -math.pi/6]
    else:
        return [get_pf_magnitude_constant(wall_distance), 0]  # x, y (robot)

def obstacle_force(front_distance):
    if front_distance == -1:
        return [0,0]
    else:
        return [-get_pf_magnitude_exponential(front_distance), math.pi/3]  # x, y (robot)

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
    wall_found = False

    #######################################
    # LAB 3 VARIABLE DECLARATION CODE : END
    #######################################

    while not rospy.is_shutdown():

        ###############################
        # LAB 3 INNER LOOP CODE : BEGIN
        ###############################

        # PART C CODE HERE:
        # Make sure that twist gets set with your drive command

        # print "wall_distance: ", wall_distance, " front_distance: ", front_distance

        wall_vector = wall_force(wall_distance)

        obstacle_vector = obstacle_force(front_distance)

        total_force = add_forces(wall_vector, obstacle_vector)

        if total_force != [0, 0]:
            wall_found = True
        else:
            wall_found = False

        if not wall_found:
            total_force = [1, 0]

        print "obstacle_vector: [%.2f, %.2f]" % (obstacle_vector[0], obstacle_vector[1]), \
        "wall_vector: [%.2f, %.2f]" % (wall_vector[0], wall_vector[1]), \
        "total_force: %.2f, %.2f" % (total_force[0], total_force[1])

        twist = drive_from_force(total_force)

        #############################
        # LAB 3 INNER LOOP CODE : END
        #############################

        #Publish drive command
        pub.publish(twist)
        rate.sleep() #Sleep until the next time to publish

    twist = Twist()
    pub.publish(twist)
