#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math

import copy

from random import randint, random
from likelihood_field import LikelihoodField


# Borrowed from class 6 starter code
def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 5000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None

        # Initialize likelihood field (code borrowed from in-class assignment)
        self.likelihood_field = LikelihoodField()

        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()


        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True



    def get_map(self, data):
        self.map = data

    def initialize_particle_cloud(self):
        # Get map origin coordiates and map resolution data
        map_data = np.array(list(self.map.data))
        resolution = self.map.info.resolution
        x_origin = self.map.info.origin.position.x
        y_origin = self.map.info.origin.position.y
        
        # Get list of valid point coordinates for the particle cloud (points inside of the room)
        valid_points = np.where(map_data == 0)[0]
        
        # Particle cloud is a random selection over valid point coordinates
        pc = np.random.choice(valid_points, self.num_particles, False)

        # For each coordinate in particle cloud, convert to 2D coordinates
        # and sample a random angle for orientation.
        for i in pc:
            pose = Pose()
            pose.position.x = (i % self.map.info.width) * resolution + x_origin
            pose.position.y = (i // self.map.info.height) * resolution + y_origin
            pose.position.z = 0
            theta = np.random.rand() * 2*math.pi
            x,y,z,w = quaternion_from_euler(0, 0, theta)
            pose.orientation = Quaternion(x,y,z,w)
            # Initialize to uniform weight
            weight = 1 / self.num_particles
            self.particle_cloud.append(Particle(pose, weight))

        self.normalize_particles()
        self.publish_particle_cloud()


    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        w_sum = sum(x.w for x in self.particle_cloud)
        for x in self.particle_cloud:
            x.w /= w_sum


    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):
        # Sample particles in current particle cloud with probability equal to the particle weights
        weights = [p.w for p in self.particle_cloud]
        sample = np.random.choice(list(range(self.num_particles)), self.num_particles, replace=True, p=weights)

        # Create new particle cloud based off of samples + some noise vector.
        new_particle_cloud = []
        for idx in sample:
            new_p = copy.deepcopy(self.particle_cloud[idx])
            # Add random vector to each particle's pose - this creates noise.
            new_p.pose.position.x += np.random.normal(0,0.25)
            new_p.pose.position.y += np.random.normal(0,0.25)
            
            new_particle_cloud.append(new_p)

        self.particle_cloud = new_particle_cloud


    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        
        # Take the mean of position data
        p_xs = [p.pose.position.x for p in self.particle_cloud]
        p_ys = [p.pose.position.y for p in self.particle_cloud]

        self.robot_estimate.position.x = np.mean(p_xs)
        self.robot_estimate.position.y = np.mean(p_ys)
        self.robot_estimate.position.z = 0

        # Take the mean of orientation data
        o_x_mean = np.mean([p.pose.orientation.x for p in self.particle_cloud])
        o_y_mean = np.mean([p.pose.orientation.y for p in self.particle_cloud])
        o_z_mean = np.mean([p.pose.orientation.z for p in self.particle_cloud])
        o_w_mean = np.mean([p.pose.orientation.w for p in self.particle_cloud])
        
        self.robot_estimate.orientation = Quaternion(o_x_mean, o_y_mean, o_z_mean, o_w_mean)
        
    
    def update_particle_weights_with_measurement_model(self, data):
        K = len(data.ranges) # array of 360
        for p in self.particle_cloud:
            x = p.pose.position.x
            y = p.pose.position.y
            q = 1

            # Only measure senor data from 8 directions, 45 degrees apart (to speed up runtime)
            for k in range(0, K, 45):
                # Catch cases where sensor doesn't detect object in range (when "nan" weight returned)
                if data.ranges[k] < data.range_max:
                    theta = get_yaw_from_pose(p.pose)
                    theta_k = (2.0 * np.pi /K)*k

                    x_k = x + min(data.ranges[k], 3) * np.cos(theta + theta_k)
                    y_k = y + min(data.ranges[k], 3) * np.sin(theta + theta_k)

                    # Will return "nan" if (x_k, y_k) is out of bounds. Handle this case below.
                    dist = self.likelihood_field.get_closest_obstacle_distance(x_k, y_k)
                    
                    q *= compute_prob_zero_centered_gaussian(dist, 0.25)

            # Case if "dist" returned "nan"
            if math.isnan(q):
                p.w = 0
            else:
                p.w = q
        

    def update_particles_with_motion_model(self):
        # based on the how the robot has moved (calculated from its odometry), we'll move
        # all of the particles correspondingly
        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

        # Calculate differences between the new pose components and old pose components
        dx = curr_x - old_x
        dy = curr_y - old_y
        d_yaw = curr_yaw - old_yaw

        # Add difference in pose components to each particle's pose in particle cloud.
        for p in self.particle_cloud:
            p.pose.position.x += dx
            p.pose.position.y += dy

            p_yaw = get_yaw_from_pose(p.pose)
            p_yaw += d_yaw
            x,y,z,w = quaternion_from_euler(0,0,p_yaw)
            p.pose.orientation = Quaternion(x,y,z,w)        



if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









