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

        ps = PoseArray()

        for i in range(particles):
            p = Pose()
            p.position.x = initialpose.pose.pose.position.x + \
                random.normalvariate(0, 1)  # increase the variance
            p.position.y = initialpose.pose.pose.position.y + \
                random.normalvariate(0, 1)
            p.position.z = initialpose.pose.pose.position.z

            # p.orientation = rotateQuaternion(
            #     initialpose.pose.pose.orientation, math.radians(random.normalvariate(0, 1))

            p.orientation = rotateQuaternion(Quaternion(w=1), getHeading(
                initialpose.pose.pose.orientation) + math.radians(random.normalvariate(0, 1)))

            ps.poses.append(p)

        return ps

    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.

        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """

        updatedPoseArray = PoseArray()

        # Each particle weight percentage
        weight = []
        for i in range(len(self.particlecloud.poses)):
            weight.append(self.sensor_model.get_weight(
                scan, self.particlecloud.poses[i]))

        self.particlecloud = updatedPoseArray

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

        len_of_particles = len(self.particlecloud.poses)

        sumX = 0
        sumY = 0
        sumZ = 0
        sumOx = 0
        sumOy = 0
        sumOz = 0

        for i in range(len_of_particles):
            sumX += self.particlecloud.poses[i].position.x
            sumY += self.particlecloud.poses[i].position.y
            sumZ += self.particlecloud.poses[i].position.z
            sumOx += self.particlecloud.poses[i].orientation.x
            sumOy += self.particlecloud.poses[i].orientation.y
            sumOz += self.particlecloud.poses[i].orientation.z

        p = Pose()
        p.position.x = sumX / len_of_particles
        p.position.y = sumY / len_of_particles
        p.position.z = sumZ / len_of_particles
        p.orientation.x = sumOx / len_of_particles
        p.orientation.y = sumOy / len_of_particles
        p.orientation.z = sumOz / len_of_particles

        return p
