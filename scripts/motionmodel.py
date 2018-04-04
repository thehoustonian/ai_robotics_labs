import math
import random       
    
def sample_motion_model_velocity(u,p,params):
    #This function will return a random next location for a robot starting 
    #in pose p and using command u, according to our sampled motion model
    #where params is all the parameters of the motion model

    # u is the command that the robot was given
    #    u[0] is the velocity command (linear.x) 
    #    u[1] is the turn command (angular.z)

    # p is the starting pose, where
    #    p[0] is the x position of the pose
    #    p[1] is the y position of the pose
    #    p[2] is the orientation/theta of the pose

    # params is a list of the parameters of the motion model.  
    #  They are assumed to be at the following indices
    #       0: Influence of |linear velocity| on velocity noise
    #       1: Influence of |angular velocity| on velocity noise
    #       2: Influence of |linear velocity| on angular noise
    #       3: Influence of |angular velocity| on angular noise
    #       4: Influence of |linear velocity| on final angle noise
    #       5: Influence of |angular velocity| on final angle noise
    #       6: Delta T to be used

    # Procedure: 
    #   1. Get randomly perturbed command (close to what is in command, but with noise)
    #   
    #   2. Calculate next pose assuming that perturbed command is perfectly executed
    #        2.a.  This includes "wiggling" the robot's orientation at the end

    # GET RANDOMLY PERTURBED COMMAND
    v = random.gauss(u[0], params[0]*math.fabs(u[0]) + params[1]*math.fabs(u[1])) #Centered on velocity command with appropriate std
    w = random.gauss(u[1], params[2]*math.fabs(u[0]) + params[3]*math.fabs(u[1])) #Centered on turn command with appropriate std
    g = random.gauss(0.0, params[4]*math.fabs(u[0]) + params[5]*math.fabs(u[1])) #Centered on zero with appropriate std

    # CALCULATE NEXT POSITION ASSUMING PERFECT EXECUTION OF (v,w) with wiggle g at the end

    # If somehow w is exactly zero, handle this separately (shouldn't ever happen with randomness)
    if abs(w) == 0:
        x = p[0] + v*math.sin(p[2])
        y = p[1] + v*math.cos(p[2])
        theta = p[2]
    else:
        #Compute new location of the robot        
        x = p[0] - (v/w)*math.sin(p[2]) + (v/w)*math.sin(p[2] + w*params[6])
        y = p[1] + (v/w)*math.cos(p[2]) - (v/w)*math.cos(p[2] + w*params[6])
        theta = p[2] + w*params[6] + g*params[6]
    
    # Return the pose that we calculated
    return x,y,theta    
    
    
    
    
