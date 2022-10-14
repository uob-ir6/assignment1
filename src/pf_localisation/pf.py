from geometry_msgs.msg import Pose, PoseArray, Quaternion
from . pf_base import PFLocaliserBase
import math
import rospy

from . util import rotateQuaternion, getHeading
from random import random

from time import time


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
        self.M = 100 #number of particles, will vary and document results
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
        ps = PoseWithCovarianceStamped() #a pose variable
        i = 0
        while i != self.M:
            #iterating over number of particles
            #Will likely need to increase the variance so that particles are better scattered
            ps.pose.pose.position.x = initialpose.pose.pose.x + random.normal(0,1)
            ps.pose.pose.position.y = initialpose.pose.pose.y + random.normal(0,1)
            ps.pose.pose.position.z = initialpose.pose.pose.z + random.normal(0,1)
            yaw = getHeading(initialpose.pose.pose.orientation) 
            #get the yaw from the current orientation framework using getHeading()
            ps.pose.pose.orientation = rotateQuaternion(Quaternion(w=1.0), yaw)
            #Append new particle to the particle cloud PC
            PC.append(ps)
            #update expression
            i += 1
        #end of while loop
        #return particle cloud
        return PS
 
    
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
        PS_ = PoseArray[] #To store resampled particles

        # Step 1: Procuring Weights for each sample
        for i in range(0,self.M): 
            #iterating over particles
            #In this loop we want to procure weights for each particle
            #and append W
            w = self.sensor_model.get_weight(scan, self.particlecloud[i])
            W.append(w)

        # Step 2: Resampling particles

        # Step 2.1: Cumulative Distribution
        C = [] # Represent cdf as an array
        C[0] = W[0] # c_1 = w_1
        for i in range(1, self.M):
            #starting from 2nd particle
            C[i] = C[i-1] + W[i]

        # Step 2.2: 
        u = np.random.uniform(0, (1/self.M)) #our threshold drawn from unifrom distr.
        i = 1
        for j in range(0,self.M):
            while u > c[i]:
                i += 1
            # If we have reached the next thresholds
            PS_[j] = self.particlecloud[i] 
            #Update u
            u += double(1/M)

        #update our particle cloud
        self.particlecloud = PS_

            

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
            s_x += self.particlecloud[i].pose.pose.position.x
            s_y += self.particlecloud[i].pose.pose.position.y
            s_z += self.particlecloud[i].pose.pose.position.z
            s_yaw += getHeading(self.particlecloud[i].pose.pose.orientation)


        estimate.pose.pose.position.x = s_x/self.M
        estimate.pose.pose.position.y = s_y/self.M
        estimate.pose.pose.position.z = s_z/self.M
        estimate.pose.pose.orientation = rotateQuaternion(Quaternion(w=1.0), s_yaw/self.M)

        return estimate

