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

todo later
 
