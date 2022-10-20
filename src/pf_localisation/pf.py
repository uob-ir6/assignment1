from glob import escape
import random
import sys
from time import sleep
from tkinter import E
from geometry_msgs.msg import Pose, PoseArray, Quaternion, PoseWithCovarianceStamped
from . pf_base import PFLocaliserBase
import math
import rospy

from . util import rotateQuaternion, getHeading




class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()

        # ----- Set motion model parameters

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 10     # Number of readings to predict
        
       
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

        poseArray = PoseArray()
        numofParticles = 500
        noise_parameter = 1

        rnd = random.normalvariate(0, 1)
 

        # # Method 1 - Initilise around initialpose with gaussian noise
        for i in range(numofParticles):
            # adding gauss random noise to the 
            pose = Pose()
            pose.position.x = initialpose.pose.pose.position.x + random.gauss(0, 5)
            pose.position.y = initialpose.pose.pose.position.y + random.gauss(0, 5)
            pose.position.z = initialpose.pose.pose.position.z
            
            pose.orientation = rotateQuaternion(initialpose.pose.pose.orientation, math.radians(np.random.uniform(0, 360))) # might need to change to guassian noise

            # add the partical to the PoseArray() object
            poseArray.poses.append(pose)

        # Method 2 - Initialise Random Uniformly
        # for i in range(numofParticles):
        #     pose = Pose()
   
        #     pose.position.x = initialpose.pose.pose.position.x + random.uniform(-15,15)
        #     pose.position.y = initialpose.pose.pose.position.x + random.uniform(-15,15)
        #     pose.position.z = initialpose.pose.pose.position.z
            
        #     pose.orientation = rotateQuaternion(initialpose.pose.pose.orientation, math.radians(random.uniform(0, 360))) # might need to change to guassian noise

        #     # add the partical to the PoseArray() object
        #     poseArray.poses.append(pose)
        
        # # return the initailised particle in the form of a PoseArray() object
        return poseArray
 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.

        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        
        scatterIndex = 0

        # Weight likelihood for each particles. p(z|x)
        weights = []
        for i in range(len(self.particlecloud.poses)):
            weights.append(self.sensor_model.get_weight(scan, self.particlecloud.poses[i]))

        # Normalise the list of weights - construct the outter ring
        normaliser = 1 / (sum(weights))
        normalisedWeights = []
        for i in range(len(weights)):
            normalisedWeights.append(weights[i] * normaliser)


        numofParticles = len(self.particlecloud.poses)
        updatedPoseList = []
        updatedPoseArray = PoseArray() # this will be the new PoseArray to set the self.particlecloud

        # Systematic Resamping
        m = 3 * numofParticles # the number of particles we want, and we want the same number of particles as we initialised.
        cumSum = np.cumsum(normalisedWeights) # cumulative sum (outter ring)
        u = random.uniform(0, 1/m)

        i = 0
        j = 0

        while j < m:
            if (u <= cumSum[i]):
                updatedPoseList.append(self.particlecloud.poses[i])
                j = j + 1
                u = u + 1 / m
            else:
                i = i + 1

        # Reduce back to original particle size
        while len(updatedPoseList) > numofParticles:
            updatedPoseList.pop(random.randrange(len(updatedPoseList)))

        # Add noise - need to make this only run in certain condition, such as when the speard of the particles is high
        for i in range(len(updatedPoseList)):
            updatedPose = Pose()
            updatedPose.position.x = random.gauss(updatedPoseList[i].position.x, 0.2) #updatedPoseList[i].position.x # gauss(updatedPoseList[i].position.x, 0.1)
            updatedPose.position.y = random.gauss(updatedPoseList[i].position.y, 0.2) # updatedPoseList[i].position.y # gauss(updatedPoseList[i].position.y, 0.1)
            updatedPose.orientation = rotateQuaternion(updatedPoseList[i].orientation, math.radians(random.gauss(getHeading(updatedPoseList[i].orientation),5))) # updatedPoseList[i].orientation #updatedPoseList[i].orientation # rotateQuaternion(updatedPoseList[i].orientation, math.radians(gauss(math.degrees(getHeading(updatedPoseList[i].orientation)),0.05)))
            updatedPoseArray.poses.append(updatedPose)

        print("============================================")
        print("sum weights: ", sum(weights)/numofParticles)

        scatterConstant = 0.05
        #Scatter the particles
        if (scatterIndex % 10 == 0): # Scatter the particles every 10 iterations.

            for i in range(int(len(updatedPoseArray.poses) * scatterConstant)): # 5% scatter particles
                midx = self.estimatedpose.pose.pose.position.x
                midy = self.estimatedpose.pose.pose.position.y
                randX = random.gauss(midx, 10) # np.random.triangular(-15, midx , 15)
                randY = random.gauss(midy, 10)  # np.random.triangular(-15, midy, 15)

                for i in range(3):
                    scatterPose = Pose()
                    scatterPose.position.x = random.gauss(randX, 0.1)
                    scatterPose.position.y = random.gauss(randY, 0.1)

                    q_orig = [0,0,0,1]
                    q_orig_msg = Quaternion(q_orig[0], q_orig[1], q_orig[2], q_orig[3])
                    scatterPose.orientation = rotateQuaternion(q_orig_msg, math.radians(np.random.uniform(0, 360)))

                    updatedPoseArray.poses.pop(random.randrange(len(updatedPoseArray.poses)))
                    updatedPoseArray.poses.append(scatterPose)

            scatterIndex = scatterIndex + 1

        self.particlecloud = updatedPoseArray

            

    

    def clustering(self):

        #Implement clustering
        #Will need to discuss how we want to do this
        #Idea: 
        #1. Find the average of the particle cloud
        s_x = 0
        s_y = 0
        s_z = 0
        s_yaw = 0
        # v_x = 0
        # v_y = 0
        # v_z = 0
        # v_yaw = 0
        d_x = 0
        d_y = 0
        d_z = 0
        d_yaw = 0


        for i in range(0,self.M):
            s_x += self.particlecloud.poses[i].position.x
            s_y += self.particlecloud.poses[i].position.y
            s_z += self.particlecloud.poses[i].position.z
            s_yaw += getHeading(self.particlecloud.poses[i].orientation)
        
        s_x /= self.M
        s_y /= self.M
        s_z /= self.M
        s_yaw /= self.M

        
        
        # #2. Find the variance of the particle cloud
        # for i in range(0,self.M):
        #     v_x += (self.particlecloud.poses[i].position.x - s_x)**2
        #     v_y += (self.particlecloud.poses[i].position.y - s_y)**2
        #     v_z += (self.particlecloud.poses[i].position.z - s_z)**2
        #     v_yaw += (getHeading(self.particlecloud.poses[i].orientation) - s_yaw)**2
        # #3. If the variance is too large, resample the particle cloud
        # if v_x > 0.5 or v_y > 0.5 or v_z > 0.5 or v_yaw > 0.5:
        #     self.resample_particle_cloud()
        # #4. If the variance is too small, increase the number of particles
        # if v_x < 0.1 and v_y < 0.1 and v_z < 0.1 and v_yaw < 0.1:
        #     self.M += 10
        #     self.resample_particle_cloud()
        # #5. If the variance is just right, do nothing
        # if v_x > 0.1 and v_x < 0.5 and v_y > 0.1 and v_y < 0.5 and v_z > 0.1 and v_z < 0.5 and v_yaw > 0.1 and v_yaw < 0.5:
        #     pass



        min_dist_total = sys.maxsize
        min_dist_particle = Pose()

        #2. Find the distance of each particle to the average
        for i in range(0,self.M):   
            d_x = (self.particlecloud.poses[i].position.x - s_x) 
            d_y = (self.particlecloud.poses[i].position.y - s_y) 
            d_z =( self.particlecloud.poses[i].position.z - s_z)
            d_yaw = (getHeading(self.particlecloud.poses[i].orientation) - s_yaw)

            # print("particle[i]: ", self.particlecloud.poses[i].position.x) 

            # print("s_xyzyaw" , s_x, s_y, s_z, s_yaw)
            # print("d_x: ", d_x)
            # print("d_y: ", d_y)
   
            # print("d_yaw: ", d_yaw)
            #3. If the distance is too large, resample the particle cloud
            if d_x > 0.5 or d_y > 0.5 or d_yaw > 0.5:
                #self.resample_particle_cloud()
                #print("Resampling")
                pass
            #4. If the distance is too small, increase the number of particles
            elif d_x < 0.1 and d_y < 0.1 and d_yaw < 0.1:
                #self.M += 10
                #self.resample_particle_cloud()
                #print("Increasing number of particles")
                pass
            #5. If the distance is just right, do nothing

            if min_dist_total >= d_x + d_y + d_z + d_yaw:
                print("min dist total: ", min_dist_total)
                min_dist_total = d_x + d_y + d_z + d_yaw
                min_dist_particle = self.particlecloud.poses[i]
                #print("min_dist_particle: ", min_dist_particle)
                
        print("\nmin_dist_particle: ", min_dist_particle)
        return min_dist_particle
   
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


        estimated_pose = Pose()
        estimate = self.clustering()


        estimated_pose.position.x = estimate.position.x
        estimated_pose.position.y = estimate.position.y
        estimated_pose.position.z = estimate.position.z
        estimated_pose.orientation = estimate.orientation
        return estimated_pose




        #pick the particle with the closest distance to all other particles as the estimate
        #this is the particle with the smallest variance
        #we can also do this by finding the average of the particle cloud
        #and then finding the particle with the smallest distance to the average
        



        # estimate = Pose() 
        # s_x = 0
        # s_y = 0
        # s_z = 0
        # s_yaw = 0

        # for i in range(0,self.M):
        #     s_x += self.particlecloud.poses[i].position.x
        #     s_y += self.particlecloud.poses[i].position.y
        #     s_z += self.particlecloud.poses[i].position.z
        #     s_yaw += getHeading(self.particlecloud.poses[i].orientation)


        # estimate.position.x = s_x/self.M
        # estimate.position.y = s_y/self.M
        # estimate.position.z = s_z/self.M
        # estimate.orientation = rotateQuaternion(Quaternion(w=1.0), s_yaw/self.M)
        # print("estimate: ", estimate)
        # return estimate
