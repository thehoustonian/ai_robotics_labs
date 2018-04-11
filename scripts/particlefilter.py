#!/usr/bin/env python
#Author: Chris Archibald, for AI-Robotics Class
#Last Modified: March 29, 2018
import rospy
import numpy as np
from sklearn.cluster import MiniBatchKMeans, KMeans
from sklearn.metrics.pairwise import pairwise_distances_argmin_min
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion
import math
import random
import copy
from labutils import *
from sensormodel import *
from motionmodel import *
from discretemap import *
from distancelut import *

#####################
# BEGIN Global Variable Definitions
robot = [0,0,0]       #Robot real position (only used for display purposes)
got_laser = False     #Don't start until we get the laser
laser_data = None     #The current laser scan (and info)
particles = []        #Our list of particles
command = None        #Current drive command to the robot
delta_t = 0.5         #Keep track of how long between predictions
delta_t_sum = 0.0     #How long between predictions as an average of previous iteration lengths
delta_t_num = 0       #How many previous iterations went into the average
distance_LUT = None   #This is our look-up table for distances from obstacles
# END Global Variable Definitions
#####################

##########################
# BEGIN ROS Topic Callback Functions [DON'T MESS WITH THIS STUFF]
##########################

#Laser callback function, store as global variable
def laserCallback(data):
    global laser_data
    global got_laser
    laser_data = data
    got_laser = True

#Command callback, store as global variable
def commandCallback(data):
    global command
    command = data

#Robot position callback, extract pose and save as global
def robotCallback(data):
    #This function updates the robots position and yaw, based on the ground truth (this is simply to display the true robot location on the images)
    global robot
    [r,p,yaw] = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    robot = [data.pose.pose.position.x,data.pose.pose.position.y,yaw]

############################
##### END CALLBACK FUNCTIONS
############################

##########################
# BEGIN PARTICLE FILTER CODE
##########################

#This class keeps track of our info for a single particle
class Particle:
    def __init__(self,x,y,theta,w=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.w = w

    def display(self):
        print ' (',self.x,',',self.y,',',self.theta,')'


#Initialize the global list of particles with n random particles
def initialize_particles(n):
    global particles
    for i in range(0, n):
        particles.append(get_random_particle())

    ##### TODO
    #TASK 1: INITIALIZE n PARTICLES TO A RANDOM LOCATION ON THE BOARD
    # particles is a list of particles (initially empty) it should be appended to
    # TIP: get_random_particle() is a function (defined) below, that will return a
    # particle at a random location on the map



#This will create and return a random valid particle (on the map, not on an obstacle)
def get_random_particle():
    global discrete_map

    #Declare variables that we will need
    px = 0
    py = 0
    ptheta = 0
    epsilon = 0.25 #Distance from edge of map not to create particles in
    valid = False

    #Keep generating random particles until one is valid
    while not valid:

        #Randomly generate x, y, and theta for this particle
        px = random.uniform(-discrete_map.map_width/2+epsilon,discrete_map.map_width/2-epsilon)
        py = random.uniform(-discrete_map.map_height/2+epsilon,discrete_map.map_height/2-epsilon)
        ptheta = random.uniform(0,2*math.pi)
        p = Particle(px,py,ptheta,0.0)

        #Check this particle in a valid location?
        if particle_on_map(p):
            valid = True

    #Found a valid particle, return it
    # Weight is initialized to -1.0 so that we know this is random
    # This is used to distinguish random particles and resampled particles for the display
    p = Particle(px,py,ptheta,-1.0)
    return p

#Advance particles forward in delta_t in time, using motion model
def advance_particles():
    global particles
    global delta_t
    global command

    #Gather control for motion model to use
    u = [command.linear.x, command.angular.z]

    #Specify parameters for motion model to use
    #  You can change these to get better performance:
    #  Following should be at these indices
    #       0: Influence of |linear velocity| on velocity noise
    #       1: Influence of |angular velocity| on velocity noise
    #       2: Influence of |linear velocity| on angular noise
    #       3: Influence of |angular velocity| on angular noise
    #       4: Influence of |linear velocity| on final angle noise
    #       5: Influence of |angular velocity| on final angle noise
    #       6: Delta T to be used
    vp = [0.14,0.11,0.2,0.18,0.1,0.1,delta_t]

    #Advance each particle
    for i in range(len(particles)):
        #Collect this particle's pose to pass into the motion model
        start_pose = [particles[i].x, particles[i].y, particles[i].theta]

        #Sample motion model to get new pose
        new_x, new_y, new_theta = sample_motion_model_velocity(u, start_pose, vp)

        #Assign this new pose to the particle
        particles[i].x = new_x
        particles[i].y = new_y
        particles[i].theta = new_theta

#Check to see if a particle is on the map
def particle_on_map(p):
    global discrete_map
    epsilon = 0.05

    if p.x < -discrete_map.map_width/2.0 + epsilon or p.x > discrete_map.map_width/2.0 - epsilon:
        return False
    if p.y < -discrete_map.map_height/2.0 + epsilon or p.y > discrete_map.map_height/2.0 - epsilon:
        return False

    (gx,gy) = discrete_map.map_to_grid((p.x,p.y))
    if (gx,gy) in discrete_map.occupied:
        return False

    return True

#Assign each particle its weight
def get_particle_weights():
    global particles

    #Cycle through each particle and assign it a weight
    #The weight should be stored in particles[i].w
    for i in range(len(particles)):
        #See if particle is in an obstacle or off the map, if it is, it gets weight of 0.0
        if particle_on_map(particles[i]):
            #ASSIGN PARTICLE i its WEIGHT (save in particles[i].w)
            particles[i].w = get_scan_prob(particles[i])
        else:
            particles[i].w = 0.0

#Get the probability of the current laser scan, for particle p
def get_scan_prob(p):
    global laser_data
    global distance_LUT

    ##### TODO
    # TASK 2: GET AND RETURN THE PROBABILITY OF THE CURRENT LASER SCAN
    # stored in (laser_data) for the input particle p
    # TIP:   particle p has fields
    #        p.x, p.y, p.theta
    #    The first three are the pose for this particle
    # TIP: To get the expected distance, call distance_LUT.distance(x,y,angle), where x,y is the location to start from, and angle is the map angle direction to travel.  This will return the distance from the look-up-table (LUT) that we expect an obstacle to be.
    # TIP: To use the sensor model, simply call sensor_model(expected_distance, measured_distance, maximum_range, std)
    # where expected distance is the distance to the expected obstacle, measured_distance is the distance that the laser measured
    # (stored in laser_data.ranges[]), and maximum_range is the maximum range reading of the laser scanner (stored in laser_data.range_max)
    # and std is the standard deviation to use for the reading noise
    sensor_model_std = 2.25 #You can modify this if it helps and use it when you call the sensor_model
    probability = 1.0
    #This variable will have the angle from the robot that the current scan is pointing
    #TIP: To get global angle to pass to LUT from this, do wrap_angle(current_angle + p.theta), where p.theta is the heading of the current particle
    current_angle = laser_data.angle_min

    for i in range(len(laser_data.ranges)):
        #IN THIS LOOP, PROCESS THE INDIVIDUAL LASER SCANS, USING THE SENSOR MODEL
        expected_distance = distance_LUT.distance(p.x, p.y, wrap_angle(current_angle + p.theta))
        sensor_model_prob = sensor_model(expected_distance, laser_data.ranges[i], laser_data.range_max, sensor_model_std)
        probability *= sensor_model_prob

        current_angle = current_angle + laser_data.angle_increment

    #RETURN THE TOTAL SCAN PROBABILITY
    return probability #CHANGE THIS LINE

#Get the next set of particles
#By resampling current ones according to weights, with replacement
def resample_particles():
    global particles

    new_particles = []
    total_weight = 0
    random_particle_count = (int)(0.50 * len(particles)) # percentage randomness of total particle count
    if(random_particle_count > len(particles)):
        print "Too many random particles!! Reducing to 1/2 total particle count"
        random_particle_count = len(particles) // 2

    #### TODO
    # TASK 3: RESAMPLE PARTICLES
    #
    # 1. Resample from old particles
    #  Using the weights of the particles, resample a new set of particles,
    #  Each time you choose one from the old set (particles), create a new one and add it to the
    #  new_particles list (make sure you create a new one with the same values, not just a reference
    #  to the old one) .  The weight for each of these new particles should be 0.0
    # 2. Add in Random Particles
    #  Make sure that you also add in some number of random particles at each iteration (you can play with how many)
    #  Look at the initialize particles code to see how to get a random particles easily
    #  The weight for each of these particles should be -1.0  (this helps us tell them apart from the
    #  resampled particles in the display code)
    #
    # 3. For the assignment the new set of particles should be the same size
    # (Some extra credit parts may require changing this)
    # Make sure that when this function exits the "particles" list is pointing to the list
    #  of the new particles

    # compute the total weight
    for particle in particles:
        total_weight += particle.w
    # print total_weight

    # Resample particles based on weight
    for i in range(0, len(particles) - random_particle_count):
        rand_weight = random.uniform(0, total_weight)
        inc_weight = 0
        particle_index = 0

        while inc_weight < rand_weight:
            inc_weight += particles[particle_index].w
            particle_index += 1

        if particle_index != 0:
            particle_index -= 1

        new_particles.append(Particle(particles[particle_index].x, particles[particle_index].y, particles[particle_index].theta, 0.0))

    # Add in random particles
    for i in range(0, random_particle_count):
        new_particles.append(get_random_particle())

    particles = copy.deepcopy(new_particles)

    # print "Particle number: " + str(len(particles))

# Get a single pose estimate from our particle filter.  This
# should be the "current best guess" according to current beliefs
# It will be displayed on the output images in black
def get_pose_estimate():
    ##### TODO
    global particles
    # TASK 4. Get a pose estimate from our beliefs
    # This code should look at all of the particles and return a single pose that
    # is the "best guess" of where the robot is right now, if we had to choose one place
    # Make sure that this gets returned as x,y,theta
    #   TIP: You can tell the difference between resampled particles and random particles by their
    #   weight.  Random particles have weight -1.0, while resampled particles have weight 0.0
    #   You probably shouldn't use the random particles to influence your pose estimate

    k_means = MiniBatchKMeans(n_clusters=5)
    coord_array = []
    guess = [0, 0, 0]
    for particle in particles:
        if particle.w == -1:
            continue
        else:
            coord_array.append([particle.x, particle.y, particle.theta])

    if len(coord_array) != 0:
        numpy_array = np.array(coord_array)
        k_means.fit(numpy_array)
        point_closest, _ = pairwise_distances_argmin_min(k_means.cluster_centers_, numpy_array)
        cluster_labels = k_means.labels_
        labels, counts = np.unique(cluster_labels[cluster_labels>=0], return_counts=True)
        #guess = k_means.cluster_centers_[labels[np.argsort(-counts)[:3]][0]]
        guess = numpy_array[point_closest[labels[np.argsort(-counts)[:3]][0]]]

    return guess[0], guess[1], guess[2]


#Update all the particles, done once per iteration
def update_particles(iteration, saveFigs):

    # 1. Motion Update: Advance physics
    advance_particles()

    if saveFigs:
        sname = discrete_map.world_dir + '/images/' + 'pf_' + discrete_map.world_name + '_' + str(iteration).zfill(4) + '_after_motion.png'
        discrete_map.display_particles(particles, robot, laser_data, get_pose_estimate(), sname)

    # 2. Sensor Update: Get weights from sensor model
    get_particle_weights()

    # 3. Normalization: Resample according to weights
    resample_particles()

    if saveFigs:
        sname = discrete_map.world_dir + '/images/' + 'pf_' + discrete_map.world_name + '_' + str(iteration).zfill(4) + '_after_resample.png'
        discrete_map.display_particles(particles, robot, laser_data, get_pose_estimate(), sname)


#Main loop
if __name__ == '__main__':

    #Initialize the ros node
    rospy.init_node('lab4pf', anonymous=True)

    #Subscribe to the topics we need
    rospy.Subscriber("base_pose_ground_truth", Odometry, robotCallback) #Subscribe to the robot pose topic
    rospy.Subscriber("base_scan_1", LaserScan, laserCallback) #Subscribe to laser scan
    rospy.Subscriber("cmd_vel",Twist,commandCallback) #Subscribe to the command issued

    #Declare needed global variables
    global delta_t
    global distance_LUT
    global discrete_map
    global robot
    global laser_data

    #Process arguments and world file name
    args = rospy.myargv(argv=sys.argv)

    #This creates the discretized map that is used to calculate expected sensor readings
    #  Resolution is the second argument, (initially 5)
    #  Feel free to change if you need to, but then the LUT for the new resolution will have
    #  to be regenerated, and this can take 30-40 minutes depending on resolution
    #  LUT for resolution 5 and the maps we use for the lab are included in the assignment
    discrete_map = DiscreteMap(args[1], 5)

    # create the distance look-up-table (LUT) to use to get expected sensor readings
    distance_LUT = DistanceLUT(discrete_map)

    #Initialize timing sum
    delta_t_sum = 0.0

    #Initialize particles
    #We will start with 500 particles, but you can experiment with different numbers of particles
    #The more the merrier, but your computer has to be able to handle it
    initialize_particles(800)

    #Set iteration counter
    iteration = 1

    #This controls how frequently we save images of the particles to the world folder
    #Change this to influence how many images get saved out
    #1 is useful for debugging, but saves a lot of images
    display_rate = 2

    #Display initial distribution of particles, if we have any
    if len(particles) > 0:
        sname = discrete_map.world_dir + '/images/' + 'pf_' + discrete_map.world_name + '_0_Initial.png'
        discrete_map.display_particles(particles, robot, laser_data, get_pose_estimate(), sname)

    #Main filtering loop
    while not rospy.is_shutdown():

        #Only proceed if we have received a laser scan
        if got_laser:

            #Store time to keep track of how long this iteration takes
            before = rospy.get_rostime()

            #Should we save out images on this iteration?
            saveFigs = True
            if (iteration % display_rate) == 0 and len(particles) > 0:
                saveFigs = True

            #Update the particles. This is the main localization code
            update_particles(iteration, saveFigs)

            #Display info for this iteration
            print '[', iteration, '] : dt = ', delta_t

            #Increment our iteration counter
            iteration = iteration + 1

            #Figure our how long iteration took, keep track of stats
            #This gives us our delta_t for prediction step
            #Probably don't modify this
            after = rospy.get_rostime()
            duration = after - before
            cur_delta_t = duration.to_sec()
            delta_t_sum = delta_t_sum + cur_delta_t
            delta_t = delta_t_sum / float(iteration)
