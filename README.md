# OMCoRPRequisites

1.	Python3
2.	Used Libraries
	
    1. joblib 
    2. multiprocessing 
    3. gc 
    4. random 
    5. numpy 
    6. networkx 
    7. matplotlib.pyplot  


A. To run the application

1. Go to the directory omcorp_and_others
2. Open a command prompt
3. Type python3 planning_application_main.py 


B. To change different parameters of the planning framework open config.py  in an editor and change accordingly. 

File Name ------------------------------------------------------------------Description
planning_application_main.py -----------------------------------------------The main file, which calls different Path Planners 
auction_application.py -----------------------------------------------------Simulate the OMCORP mechanism
colors.py ------------------------------------------------------------------Helper Class
config_values.py -----------------------------------------------------------Stores different parameters for the path planning
generate_workspace.py ------------------------------------------------------Generates the workspace and the graphs to represent the workspace from                                                                                             the given specifications, stored in config_values.py
mobile_robot.py ------------------------------------------------------------Simulates the behavior of a robot
mstar_application2.py ------------------------------------------------------Generates paths for robots by using M*
prioritized_planing_application_3.py ---------------------------------------Generates paths for robots by using prioritized planning
workspace_graph.py ---------------------------------------------------------Helper class for M*
SortedCollection.py --------------------------------------------------------Helper class for M*
interface.py ---------------------------------------------------------------Helper class for M*
od_mstar.py ----------------------------------------------------------------Helper class for M*
col_set_addition.py --------------------------------------------------------Helper class for M*

