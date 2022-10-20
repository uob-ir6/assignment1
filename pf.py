from geometry_msgs.msg import Pose, PoseArray, Quaternion, PoseWithCovarianceStamped
from . pf_base import PFLocaliserBase
import math
import rospy
import numpy as np
from . util import rotateQuaternion, getHeading
from random import random, uniform, gauss
import random

from time import sleep, time


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.05
        self.ODOM_TRANSLATION_NOISE = 0.05
        self.ODOM_DRIFT_NOISE = 0.05
        
        self.M = 250 #number of particles, will vary and document results
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        
       
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        PC = PoseArray() #particle cloud
        #a pose variable
        i = 0
        
        #when initial pose is given in the bag file! (otherwise see below)
        while i != self.M:
            ps = Pose() 
            #iterating over number of particles
            #Will likely need to increase the variance so that particles are better scattered
            ps.position.x = initialpose.pose.pose.position.x + np.random.normal(0,1)
            ps.position.y = initialpose.pose.pose.position.y + np.random.normal(0,1)
            ps.position.z = initialpose.pose.pose.position.z 
            yaw = getHeading(initialpose.pose.pose.orientation) 
            #get the yaw from the current orientation framework using getHeading()
            #adding noise
            yaw = yaw + math.radians(np.random.uniform(0,360))
            ps.orientation = rotateQuaternion(Quaternion(w=1.0), yaw)
            #Append new particle to the particle cloud PC
            PC.poses.append(ps)
            #update expression
            i += 1
        #end of while loop
        
        #when initial pose is not given in bag file; initialize randomly
        '''
        while i != self.M:
            ps = Pose()
            ps.position.x = np.random.uniform(-20, -20)
            ps.position.y = np.random.uniform(-20, -20)
            ps.position.z = 0
            randang = math.radians(np.random.uniform(0,360))
            ps.orientation = rotateQuaternion(Quaternion(w=1.0), randang)
            PC.poses.append(ps)
            i += 1
        '''
        #return particle cloud
        return PC
 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        '''
        We will need to use the methods of the sensor model to generate weights,
        particularly, get_weight()

        '''
        W = [] #array of weights (doubles returned by get_weight())
        PS_ = [] #To store resampled particles (since PoseArray() is not subscriptable)
        PS = PoseArray()
        scatterIndex = 0

        # Step 1: Procuring Weights for each sample
        for i in range(0,self.M): 
            #print(i)
            #iterating over particles
            #In this loop we want to procure weights for each particle
            #and append W
            w = self.sensor_model.get_weight(scan, self.particlecloud.poses[i])
            W.append(w)#not normalised weights
            
        n = 1 / sum(W) # normalizer
        for i in range(0,self.M):
            W[i] = W[i] * n

        # Step 2: Resampling particles

        # Step 2.1: Cumulative Distribution
        C = [0] # Represent cdf as an array

        C[0] = W[0] # c_1 = w_1
        for i in range(1, self.M):
            #starting from 2nd particle
            C.append(C[i-1] + W[i]) # c_i = c_(i-1) + w_i

        # Step 2.2: 
        u = np.random.uniform(0, (1/self.M)) #our threshold drawn from unifrom distr.
        i = 0

        for j in range(0,self.M):
            while u > C[i]:
                i += 1
            # If we have reached the next thresholds
            PS_.append(self.particlecloud.poses[i]) 
            #Update u
            u += 1/self.M
            
        print("============================================")
        #'''
        #Scatter the particles otherwise we will have too many particles of same value
        for i in range(0, 20):
        	p = Pose()
        	p.position.x = np.random.uniform(self.estimatedpose.pose.pose.position.x-10, self.estimatedpose.pose.pose.position.x+10)
        	p.position.y = np.random.uniform(self.estimatedpose.pose.pose.position.y-10, self.estimatedpose.pose.pose.position.y+10)
        	p.position.z = 0
        	p.orientation = rotateQuaternion(Quaternion(w=1.0), math.radians(np.random.uniform(0, 360)))
        	PS.poses.append(p)
        for i in range(20,self.M):
            varP = Pose()
            varP.position.x = gauss(PS_[i].position.x, 0.2)
            varP.position.y = gauss(PS_[i].position.y, 0.2)
            varP.position.z = 0
            yaw = getHeading(PS_[i].orientation)
            varP.orientation = rotateQuaternion(PS_[i].orientation, math.radians(gauss(yaw, 5)))
            PS.poses.append(varP)
            


        '''
        if (scatterIndex % 10 == 0): # Scatter the particles every 10 iterations.

            for i in range(int(len(PS.poses) * 0.01)): # 5% scatter particles

                randX = gauss(self.estimatedpose.pose.pose.position.x, 15)
                randY = gauss(self.estimatedpose.pose.pose.position.y, 15)

                for i in range(3):
                    scatterPose = Pose()
                    scatterPose.position.x = gauss(randX, 0.1)
                    scatterPose.position.y = gauss(randY, 0.1)

                    q_orig = [0,0,0,1]
                    q_orig_msg = Quaternion(q_orig[0], q_orig[1], q_orig[2], q_orig[3])
                    scatterPose.orientation = rotateQuaternion(q_orig_msg, math.radians(np.random.uniform(0, 360)))

                    PS.poses.pop(random.randrange(self.M))
                    PS.poses.append(scatterPose)

            scatterIndex = scatterIndex + 1
        
            
        


        for i in range(0, 20):
        	p = Pose()
        	p.position.x = np.random.uniform(self.estimatedpose.pose.pose.position.x-10, self.estimatedpose.pose.pose.position.x+10)
        	p.position.y = np.random.uniform(self.estimatedpose.pose.pose.position.y-10, self.estimatedpose.pose.pose.position.y+10)
        	p.position.z = 0
        	p.orientation = rotateQuaternion(Quaternion(w=1.0), math.radians(np.random.uniform(0, 360)))
        	PS.poses.append(p)
        	self.M += 1
        self.particlecloud = PS
        '''


        #update our particle cloud
        self.particlecloud = PS

            

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
        """

        #Implementing average for now
        #We can discuss and try clustering later
        estimate = Pose() 
        s_x = 0
        s_y = 0
        s_z = 0
        s_yaw = 0

        for i in range(0,self.M):
            s_x += self.particlecloud.poses[i].position.x
            s_y += self.particlecloud.poses[i].position.y
            s_z += self.particlecloud.poses[i].position.z
            s_yaw += getHeading(self.particlecloud.poses[i].orientation)


        estimate.position.x = s_x/self.M
        estimate.position.y = s_y/self.M
        estimate.position.z = s_z/self.M
        estimate.orientation = rotateQuaternion(Quaternion(w=1.0), s_yaw/self.M)

        return estimate

