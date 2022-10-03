# Multiple Objective Particle Swarm Optimization (MOPSO)

-----------

## Author: Mahsa Sheikhihafshejani - msheikhihafshejani@smu.edu

-----------
This code optimizes a non-linear multi-objective mixed integer model for a sustainable Location Inventory Routing Problem (LIRP) considering multiple trips, vehicle speed, analysis costs,
the injury rate of vehicles using MOPSO metaheuristics

Following are the steps of the algorithm:
1. Initialize the swarm & archive
2. For each particle in the swarm:
    A. Select leader from the archive
    B. Update velocity
C. Update position
3. Update the archive of non-dominated solutions
4. Repeat
