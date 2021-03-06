#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion
import math

#####################
# BEGIN Global Variable Definitions

robot = [0,0,0]
laser_scan = None
goal = None

# END Global Variable Definitions
#####################+

#####################
# BEGIN ROS Topic Callback Functions [DON'T MESS WITH THIS STUFF]
#####################

def robot_callback(data):
    #This function updates the robots position and yaw, based on the ground truth (we don't have localization yet)
    global robot
    [r,p,yaw] = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    robot = [data.pose.pose.position.x,data.pose.pose.position.y,yaw]

def laser_callback(data):
    #This function sets the global laser_scan variable to hold the most recent laser scan data
    global laser_scan
    laser_scan = data

def goalCallback(data):
    #This function will update the goal of the robot
    global goal
    goal = [data.x, data.y]

#####################
##### END CALLBACK FUNCTIONS
#####################

#####################
# BEGIN HELPER FUNCTIONS [YOU CAN USE THESE IN YOUR CODE, BUT YOU SHOULDN'T NEED TO MODIFY]
#####################

def add_forces(a, b):
    #This function adds two force vectors together and returns the result
    assert len(a) == len(b), "Force vectors differ in length"
    c = [a[i] + b[i] for i in range(len(a))]
    return c

def wrap_angle(angle):
    #This function will take any angle and wrap it into the range [-pi, pi]
    while angle >= math.pi:
        angle = angle - 2*math.pi

    while angle <= -math.pi:
        angle = angle + 2*math.pi
    return angle

#####################
##### END HELPER FUNCTIONS
#####################

#####################
##### TREY'S HELPERS
####################

def mag_to_xy(vector):
    # vector = [magnitude, direction]
    # converts a vector represented by magnitude and direction and returns an [x, y] force
    x = vector[0]*math.cos(vector[1])
    y = vector[0]*math.sin(vector[1])
    return [x, y]

def xy_to_mag(vector):
    # vector = [x, y]
    # converts a vector from [x,y] representation to [mag, dir] representation
    direction = math.atan2(vector[1], vector[0])
    magnitude = math.hypot(vector[0], vector[1])
    return [magnitude, direction] # [magnitude, direction]

def compute_vector(robot, goal_position):
    # computes the direction of the vector towards the goal (GLOBAL)
    diff_x = goal_position[0] - robot[0]
    diff_y = goal_position[1] - robot[1]
    angle = math.atan2(diff_y, diff_x)

    # have to do this to convert between global and local (ROBOT-CENTRIC) coordinates
    direction = wrap_angle(angle - robot[2])
    magnitude = math.hypot(diff_x, diff_y)

    # print "diff_x: ", diff_x, " diff_y: ", diff_y, " direction: ", direction, " magnitude: ", magnitude, " mag_to_xy: ", mag_to_xy([magnitude, direction])

    return [magnitude, direction]


#####################
##### END TREY'S HELPERS
####################

#####################
# BEGIN MODIFIABLE LAB CODE [ALTHOUGH MOST MODIFICATIONS SHOULD BE WHERE SPECIFIED]
#####################

#This function takes in a force [x,y] (in robot coordinates) and returns the drive command (Twist) that should be sent to the robot motors
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

# This function determines and returns the attractive force (force_to_goal) to the goal.
# This force should be in robot coordinates
def goal_force( ):

    #This is the robot's actual global location, set in robot_callback
    global robot #format [x_position, y_position, yaw]

    #Goal location is in the global 'goal' variable

    #####################################################
    #PARAMETERS : MODIFY TO GET ROBOT TO MOVE EFFECTIVELY

    #Parameter : MODIFY
    #This should be used to scale the magnitude of the attractive goal force
    # strength = 1.0  # extra credit pt 4
    strength = 0.75

    #END OF PARAMETERS
    #####################################################

    force_to_goal = [0,0]

    #########################
    # LAB 2 PART A : BEGIN
    #########################

    # PART A CODE HERE:
    #    1. Compute goal force vector and put it in the 'force_to_goal' variable
    force_mag = compute_vector(robot, goal)
    force_to_goal= mag_to_xy([force_mag[0]*strength + 1, force_mag[1]]) # robot-centric

    # for extra credit pt 4:
    # force_to_goal = mag_to_xy([8, force_mag[1]])

    #########################
    # LAB 2 PART A : END
    #########################

    return force_to_goal


#This function looks at the current laser reading, then computes and returns the obstacle avoidance force vector (in local robot coordinates)
def obstacle_force():

    #The most recent laser_scan.  It has the following fields
    #   laser_scan.angle_min : angle of the first distance reading
    #   laser_scan.angle_increment : the angular difference between consecutive distance readings
    #   laser_scan.ranges : an array all of the distance readings
    global laser_scan

    #Only run if we have a laser scan to work with
    if laser_scan is None:
        return [0,0]

    #The obstacle repulsion force variable, will be returned
    #This will accumulate all of the obstacle forces acting upon us
    force_from_obstacles = [0,0]

    cur_angle = laser_scan.angle_min
    #cur_angle will always have the relative angle between the robot's yaw and the current laser reading

    for i in range(len(laser_scan.ranges)):

        # Get the magnitude of the repulsive force for this distance reading
        # CHANGE WHICH FUNCTION IS CALLED FOR LAB 2 PART C

        # For extra credit part 2 tangent + linear repulsion:
        """
        strength = get_pf_magnitude_linear(laser_scan.ranges[i]) * 3
        tan_strength = get_pf_magnitude_constant(laser_scan.ranges[i]) * 2
        """

        # normal:
        # strength = get_pf_magnitude_linear(laser_scan.ranges[i])

        # exponential:
        strength = get_pf_magnitude_exponential(laser_scan.ranges[i])

        #########################
        # LAB 2 PART B : BEGIN
        #########################

        # PART B CODE HERE:
        #    1. Compute force vector with magnitude 'strength' away from obstacle
        #    2. Add this force vector to the 'force_from_obstacles' vector

        # for extra credit pt 2:
        """
        obstacle_vector = mag_to_xy([-strength, cur_angle])
        tan_vector = mag_to_xy([-tan_strength, cur_angle + math.pi/2])
        force_from_obstacles[0] += obstacle_vector[0] + tan_vector[0]
        force_from_obstacles[1] += obstacle_vector[1] + tan_vector[1]
        """
        # normal:

        obstacle_vector = mag_to_xy([-strength, cur_angle])
        force_from_obstacles[0] += obstacle_vector[0]
        force_from_obstacles[1] += obstacle_vector[1]

        # extra credit pt 4:
        """
        direction = cur_angle + math.pi/2

        obstacle_vector = mag_to_xy([strength, direction])
        force_from_obstacles[0] += obstacle_vector[0]
        force_from_obstacles[1] += obstacle_vector[1]
        """
        #########################
        # LAB 2 PART B : END
        #########################

        cur_angle = cur_angle + laser_scan.angle_increment

    return force_from_obstacles

# This function returns the magnitude of repulsive force for the input distance
# using a linear drop-off function
def get_pf_magnitude_linear(distance):

    #####################################################
    #PARAMETERS: MODIFY TO GET THINGS WORKING EFFECTIVELY

    #How close to the obstacle do we have to be to begin feeling repulsive force
    distance_threshold = 0.8

    #The maximum strength of the repulsive force
    max_strength = 0.75

    #END OF PARAMETERS
    #####################################################

    #########################
    # LAB 2 PART C : BEGIN
    #########################

    # PART C CODE HERE:
    #   1. Compute the magnitude of the force for the given distance and return it

    if distance <= distance_threshold:
        strength = ((distance_threshold - distance) / distance_threshold) * max_strength
    else:
        strength = 0

    #########################
    # LAB 2 PART C : END
    #########################

    return strength #CHANGE TO RETURN THE VALUE YOU COMPUTE

# This function returns the magnitude of repulsive force for the input distance
# using a constant value if the obstacles is closer than a threshold
def get_pf_magnitude_constant(distance):

    #####################################################
    #PARAMETERS: MODIFY TO GET THINGS WORKING EFFECTIVELY

    #How close to the obstacle do we have to be to begin feeling repulsive force
    distance_threshold = 0.85

    #Strength of the repulsive force
    strength = 1.0

    #END OF PARAMETERS
    #####################################################

    if distance < distance_threshold:
        return strength

    return 0

# This function returns the magnitude of repulsive force for the input distance
# using an exponential decay function.
def get_pf_magnitude_exponential(distance):
    distance_mult = 9  # used to scale the exponential force so it's not as strong far away
    # distance_mult = 4  # extra credit pt 4
    distance *= distance_mult
    strength = 1/(distance * distance)
    return strength

# This is the main loop of the lab code.  It runs continuously, navigating our robot
# (hopefully) towards the goal, without hitting any obstacles
def potential():
    rospy.init_node('lab2', anonymous=True) #Initialize the ros node
    pub = rospy.Publisher('cmd_vel', Twist) #Create our publisher to send drive commands to the robot
    rospy.Subscriber("base_scan", LaserScan, laser_callback) #Subscribe to the laser scan topic
    rospy.Subscriber("base_pose_ground_truth", Odometry, robot_callback) #Subscribe to the robot pose topic
    rospy.Subscriber("next_waypoint", Point, goalCallback)#Subscribe to the goal location topic

    rate = rospy.Rate(10) #10 Hz

    while not rospy.is_shutdown():

        #Don't do anything until the goal location has been received
        if goal is None:
            rate.sleep()
            continue

        #1. Compute attractive force to goal
        g_force = goal_force()

        #2. Compute obstacle avoidance force
        o_force = obstacle_force()

        # print "g_force: ", g_force, " o_force: ", o_force
        #3. Get total force by adding together
        total_force = add_forces(g_force, o_force)

        #4. Get final drive command from total force
        twist = drive_from_force(total_force)

        #5. Publish drive command, then sleep
        pub.publish(twist)

        rate.sleep() #sleep until the next time to publish


    #Send empty twist command to make sure robot stops
    twist = Twist()
    pub.publish(twist)

if __name__ == '__main__':
    try:
        potential()
    except rospy.ROSInterruptException:
        pass
