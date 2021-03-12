# Size of each obstacle block
obstacle_block_size = 3

# Define the total size of the workspace : obstacle_block_number *(obstacle_block_size+4) +2 
obstacle_block_number = [4]

# Max Robot = 10*(obstacle_block_number * obstacle_block_number)  --- Approximated Number
robot_number=[2, 3, 4]

# Total number of iterations of the simulation
number_of_iterations_per_robot_number = 20

max_permissible_path = 1500

# Max time for Mstar
sim_time = 1200

# Make the flag to 1 for asynchronous planning, for 0, the planning is synchronous. In synchronous planning mode, all the robots arrive at time 0
# On the other hand, robots arrive at different time in asynchronous planning
flag_asynchronous_planning = 0

#Inflation level used by Mstar
inflation_level = 2

# Make the flag 1 for synchronous tarnsitions of the robots in auction planning
flag_synchronous_transition = 1
