from typing_extensions import Self
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

        particles = 100
        i = 0

        ps = PoseArray()

        while i != particles:
            p = Pose()
            p.position.x = initialpose.pose.pose.position.x + \
                random.normalvariate(0, 1)  # increase the variance
            p.position.y = initialpose.pose.pose.position.y + \
                random.normalvariate(0, 1)
            p.position.z = initialpose.pose.pose.position.z
            yaw = getHeading(initialpose.pose.pose.orientation) + \
                random.normalvariate(0, 1)
            p.orientation = rotateQuaternion(Quaternion(w=1.0), yaw)
            ps.append(p)
            i += 1

        return ps

    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.

        #initialise <particle, weight> combination as 
        pwArray = []

        //for each particle 1...m 
            //sample a particle sample xt[m] ~ p(xt Z ut, xt-1[m])
            // sampledparticle = TODO
            
            #w eight for particle m = p(z1 | xt[m]) 
            weight = self.sensor_model.get_weight(scan, sampledParticle)
            # tuple (particle, weight)
            particleWeightTuple = (sampledParticle, weight)
            # this: pwArray.append(particleWeightTuple)
            # or this: self.particlecloud.poses.append((sampledParticle, weight))

        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
         
         
        
        pass

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
        pass
