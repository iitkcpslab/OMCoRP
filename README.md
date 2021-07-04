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

File Name : Description

1. planning_application_main.py : The main file, which calls different Path Planners 

2. auction_application.py : Simulate the OMCORP mechanism 

3. colors.py : Helper Class

4. config_values.py : Stores different parameters for the path planning

5. generate_workspace.py : Generates the workspace and the graphs to represent the workspace from                                                                                             the given specifications, stored in config_values.py

6. mobile_robot.py : Simulates the behavior of a robot

7. mstar_application2.py : Generates paths for robots by using M*

8. prioritized_planing_application_3.py : Generates paths for robots by using prioritized planning

9. workspace_graph.py : Helper class for M*

10. SortedCollection.py : Helper class for M*

11. interface.py : Helper class for M*

12. od_mstar.py : Helper class for M*

13. col_set_addition.py : Helper class for M*

