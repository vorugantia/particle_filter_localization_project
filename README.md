# particle_filter_localization_project
 
## Implementation plan

Team members: Arjun Voruganti, Oscar Michel

1) `initialize_particle_cloud`: The particle cloud is represented as a list. To initialize the particle cloud, first we find the set of points that are inside the room and unoccupied by walls. Then from this set, we pick 1000 points at random and add it to our particle cloud.

2) `update_particles_with_motion_model`: For each Particle() in the particle cloud list, we want to add the displacement of the robot's movement to the particle pose. For example, the robot's displacement in the x direction is represented by (curr_x - old_x). This moves each particle to a "hypothetical" position.

3) `update_particle_weights_with_measurement_model`: For each Particle() in the particle cloud list in its "hypothetical" position, we need to measure its distance from the nearby walls according to its orientation (yaw). Then, we need to put these calculated distances into the weight formula (difference between the robot's LaserScan data and the Particle's calculated distance from walls.)

4) `normalize_particles`: Go through the particle cloud list, add up the weights for each individual Particle(), and then divide each Particle() weight by this total sum.

5) `resample_particles`: Utilizes the `draw_random_sample` helper function, which will call call `numpy.random.choice` in order to sample a list of all particles according to their normalized weights. In order to do this we have to make auxillary 1D numpy arrays of particles and particle weights, and pass them into 
`numpy.random.choice`. This will give us a new list of sampled particles, and we can set the particle cloud to that.

6) `update_estimated_robot_pose`: We want to take the average pose over all poses of the particles in the particle cloud.

7) In order to incorporate noise, we need to add a random noise vector to each particle's pose if they get chosen in the `resample_particles` step.

### Tentative timeline (assignment due 4/26):

We plan to get as much done as possible this weekend. By Sunday 4/18, we want to attempt each step and come up with a complete implementation. We are expecting bugs. Then we will spend the next week fixing these bugs that come up. We will also use this next week to find a way to get the robot out of the house.


## Writeup

### Objective: 

Implement a particle filter algorithm to localize Turtlebot using the likelihood field method. Given an arbitrary starting position in the map, the robot should be able to accurately localize itself in the room as the particle cloud converges to its true location. We use the robot's LIDAR sensor measurements and its odometry to update particle weights.

### High-level Description

We first find a list of valid positions on the map, then sample our initial cloud uniformly from this set of points. When the robot has moved far enough, the position of each particle is moved by the same amount the robot moved. Once this has been done, the particle weights can be calculated using the liklehood field method. Specifically, we measure the distance to the closest wall from eight sensor directions around the particle. If the particle's location is close to the robot, this method will give it a very high weight since its hypothetical sensor measurements will be consistent with the robot's actual sensor data. Once the weight for each particle is computed, we normalize the weights of the particle cloud so that new particles can be sampled with probability equal to its weight. After a new particle cloud has been sampled, noise is added to the x-y coordinate of the particle's position so that the particle cloud does not collapse to a single point. Rather, we want the particles to converge to a cloud centered at the robot's true location. Finally, we update the estimated position of the robot by taking a coordinate-wise mean of each particle's position and orientation vector.

### Code Description

#### Initialization of particle cloud

This is done in the `initialize_particle_cloud` method located on line 126. We first find a list of valid points where the robot could be in the room. These are the points in the occupancy grid with zero value. From this list, we sample the positions that will be in the initial point cloud using `np.random.choice`. The number of points we sample is given by `self.num_particles`. With the sampled positions, we initialize particles y_by converting the location in the occupany grid to x-y coordinates and randomly sampling an orientation angle. 


#### Movement model

This is done in the `update_particles_with_motion_model` method located on line 326. Using the robot's position measured from its odometry, we calculate the displacement from the robot's current position and orientation from its last measured position and orientation. This displacement in x, y and yaw is then added to each particles current position and orientation. For orientation, we make use of the `get_yaw_from_pose` and `quaternion_from_euler`.


#### Measurement model

We do this in `update_particle_weights_with_measurement_mode` on 297. We iterate through the particle cloud and apply the likelihood field method. For computational efficiency, we only sample eight directions of the robot's sensor measurement. For each of these directions, we compute hypothetical `x_k` and `y_k` coordinates according to the formula given in the lecture. The only difference is that our method clips the measured distance to 3 m to avoid infinities in the LIDAR measurements. We use `get_closest_obstacle_distance` to compute the final weighting. We found that a standard deviation of 0.25 works well in the Gaussian distribution used to get likelihood. 

#### Resampling/Incorporation of noise

This is done in the `resample_particles` method. We use `np.random.choice` to sample a new set of particles with probability equal to particle weight. We then add some noise to each resampled particles x-y position by sampling from `np.random.normal(0,0.25)`. We found that it is better to not add noise to the new particles orientation. 

#### Optimization of parameters

The parameters that we optimized where the number of particles and the standard deviation for the Gaussians used to add noise and to get particle likelihoods. Following the suggestion of Yves, we do not include `z_max` and `z_random` in our weight model. We experimented with each of these standard deviations by hand and found which ones worked well. The standard deviation for adding noise will affect the width of the point cloud, so if few particles are used it is better to have a large such standard deviaiton. The standard deviation used for likelihood will affect the speed of point cloud convergene, since it controls how much particle measurements are penalized for deviating from the robot's measurements.


