import sys
import od_mstar
import time
import random
import timeout_decorator

#import workspace_graph
from col_set_addition import OutOfTimeError, NoSolutionError
import time
from datetime import datetime
import numpy as np
import networkx as nx
import pandas as pd
import matplotlib.pyplot as plt
from colors import color
from mobile_robot import *
from config_values import *

#sys.stdout = open('output.txt','wt')
valid = []
matrix = []
FREE = 0
OBS = 1

       
               	        
               
def generate_Path_Mstar(workspace_x, workspace_y, init_pos, goal_pos, workspace):
    
    
    
    
    
    print("Initial positions: "+str(init_pos))
    print("Goal positions:" + str(goal_pos))
    start_time = time.time()
    try:
        
        path = od_mstar.find_path(workspace, init_pos, goal_pos, inflation=inflation_level, astar=True, time_limit=sim_time, recursive=True, connect_8=False)
        
    except OutOfTimeError:
        path = [[]]
        print("OutOfTimeError")
        tmp_line_part_1 = "Mstar Workspace " + str(workspace_x) + " by " + str(workspace_y) +" Total Robots: "+ str(len(init_pos)) 
        tmp_line_part_2 = " Time Out: " +  str(sim_time) + "\n" 
        line_total =  tmp_line_part_1 + tmp_line_part_2 
        print(line_total) 
        file_name=open('result.txt','a')
        file_name.write(line_total)
        file_name.close() 
    except NoSolutionError:
        path = [[]] 
        print("NoSolutionError")
    
    end_time = time.time()
    computation_time =  end_time - start_time
    print("Computation time: " + str(computation_time))
    

    """if path != [[]]:
        #for z in path:
            #print(z)
        #extractPath(init_pos,goals,path,)    
    else:
        print("No Path Found")"""
        
    return path, computation_time
 
class Mstar_Plan_Executor:
    def __init__(self, workspace_x, workspace_y, workspace_2, neighborDirectedGraph, workspace_1, robot_list, current_inflation_level):
        self.workspace_x =  workspace_x
        self.workspace_y =  workspace_y
        self.workspace_zero_one = workspace_2
        self.neighborDirected = neighborDirectedGraph.copy()
        self.workspace_directional = workspace_1
        self.robot_list = robot_list
        self.active_robot_list = []
        self.last_replan_time = -1.0
        self.newly_arrived_robots = []
        self.max_plan_execution_time = 0
        self.timeout_flag = 0
        self.plan_formulation_time =  []
        self.inflation_level = current_inflation_level
        self.max_path_length_allowed = int(max_permissible_path * (1+ len(self.robot_list)/250) * (1 + flag_asynchronous_planning))
        
        
    def check_collision(self):
        collision_location_dict = {}
        for tmp_robot in self.robot_list:
            tmp_location = tmp_robot.get_current_location()
            tmp_key = str(tmp_location[0]) + " " + str(tmp_location[1])
            collision_location_dict.setdefault(tmp_key,[])
            collision_location_dict[tmp_key].append(tmp_robot.id)
        for key, robot_ids in collision_location_dict.items():
            print("Key: " + str(key) + " Robot list: " +str(robot_ids))
                                      
            
        
    def check_new_arrivals(self):
        self.newly_arrived_robots.clear()
        for tmp_robot in self.robot_list:
            
            if(tmp_robot.arrival_time > self.last_replan_time and tmp_robot.arrival_time <= self.max_plan_execution_time):
                self.newly_arrived_robots.append(tmp_robot.id)
        print("New arrivals: "+ str(self.newly_arrived_robots))
        if(len(self.newly_arrived_robots) >= 2):
            
            self.last_replan_time = self.max_plan_execution_time
            
            return 1
        else:
           
            return 0
    
    @timeout_decorator.timeout(sim_time, use_signals=False)    
    def generate_Path_Mstar(self, init_pos, goal_pos):
        
        print("Initial positions: "+str(init_pos))
        print("Goal positions:" + str(goal_pos))
        start_time = time.time()
        try:
            
            path = od_mstar.find_path(self.workspace_zero_one, init_pos, goal_pos, inflation=self.inflation_level, astar=True, time_limit=sim_time, recursive=True, connect_8=False)
        
        except OutOfTimeError:
            path = [[]]
            print("OutOfTimeError")
            tmp_line_part_1 = "Mstar Workspace " + str(self.workspace_x) + " by " + str(self.workspace_y) +" Total Robots: "+ str(len(init_pos)) 
            tmp_line_part_2 = " Time Out: " +  str(sim_time) + "\n" 
            line_total =  tmp_line_part_1 + tmp_line_part_2 
            print(line_total) 
            file_name=open('result.txt','a')
            file_name.write(line_total)
            file_name.close()
            self.timeout_flag = 1
            
        except NoSolutionError:
            path = [[]] 
            print("NoSolutionError")
        
            
    
        end_time = time.time()
        computation_time =  end_time - start_time
        print("Computation time: " + str(computation_time))
        for tmp_path in path:
            print(tmp_path)
        
        return path, computation_time
        
    def extract_path(self, path):
        tmp_active_robots = len(self.active_robot_list)
        for tmp_id in range(tmp_active_robots):
            tmp_robot_goal = self.active_robot_list[tmp_id].get_goal_location()
            print("Path of Robot: "+str(self.active_robot_list[tmp_id].id)+ " Goal: "+str(tmp_robot_goal))
            tmp_robot_path = []
            for tmp_path in path:
                if(tuple(tmp_robot_goal) != tuple(tmp_path[tmp_id])):
                    tmp_robot_path.append((tmp_path[tmp_id]))
            tmp_robot_path.append(tuple(tmp_robot_goal))
            print(tmp_robot_path)
            self.active_robot_list[tmp_id].update_path_information(tmp_robot_path)
        
        
    def replan_path(self):
        
        init_pos = []
        goal_pos = []
        self.active_robot_list.clear()
        for tmp_robot in self.robot_list:
            if(tmp_robot.arrival_time <= self.max_plan_execution_time):
                tmp_current_pos = tmp_robot.get_current_location()
                tmp_goal_pos = tmp_robot.get_goal_location()
                if(tuple(tmp_goal_pos) != tuple(tmp_current_pos)):
                    init_pos.append(tmp_current_pos)
                    goal_pos.append(tmp_goal_pos)
                    self.active_robot_list.append(tmp_robot)
        try:                
            path, path_computation_time = self.generate_Path_Mstar(init_pos, goal_pos)
        except timeout_decorator.timeout_decorator.TimeoutError:
            path = [[]]
            path_computation_time = sim_time
            self.timeout_flag = 1
       
            
        self.plan_formulation_time.append(path_computation_time)
        if(path != [[]]):
            self.extract_path(path)
        else:
             self.active_robot_list.clear()
             self.timeout_flag = 1
             
        for tmp_robot in self.active_robot_list:
            print("Robot: " + str(tmp_robot.id))
            
       
                
        
    def excute_path_Mstar(self):
        
        flag = 1
        while(flag == 1 and self.max_plan_execution_time <= self.max_path_length_allowed):
            flag = 0
            self.check_collision()
            new_arrival_flag = self.check_new_arrivals()
            if(new_arrival_flag == 1):
                self.replan_path()
            #if timeout is occured, no path found    
            if(self.timeout_flag == 1):
                
                break
            for tmp_robot in self.active_robot_list:
                robot_next_location =tmp_robot.get_next_location()
                
                tmp_robot.show_status()
                if tuple(robot_next_location) != tuple([-1, -1]):
                    flag = 1
                    tmp_robot.status = 'M'
                    
            
        
            for tmp_robot in self.active_robot_list:
                tmp_robot.update_waiting_time() # Only for M* and PP Planning
                tmp_robot.update_location()
        
                   
            self.max_plan_execution_time = self.max_plan_execution_time + 1
           
            
    def get_average_path_mstar(self):
        tmp_average_path = 0
        for tmp_robot in self.robot_list:
            tmp_average_path = tmp_average_path + tmp_robot.total_path_traversed
        total_robots = len(self.robot_list)

        tmp_path_length = tmp_average_path/total_robots
        if(self.timeout_flag == 1):
            tmp_path_length = 0 
        return  (tmp_path_length)
        
    def save_wait_time(self):
        
        tmp_wait_time_all = []
        
        tmp_economy_wait_time = []
        tmp_regular_wait_time = []
        tmp_premium_wait_time = []
        
        
        for tmp_robot in self.robot_list:
            tmp_wait_time = tmp_robot.waitTime  

            if(self.timeout_flag == 1):
                tmp_wait_time = 0

          
            tmp_wait_time_all.append(tmp_wait_time)
            tmp_category = tmp_robot.category
            
 
            
            if(tmp_category == "economy"):
                tmp_economy_wait_time.append(tmp_wait_time)
                
                
            if(tmp_category == "regular"):
                tmp_regular_wait_time.append(tmp_wait_time)
                
                
            if(tmp_category == "premium"):
                tmp_premium_wait_time.append(tmp_wait_time)
                
                
        total_robots = len(self.robot_list)
        file_name = 'robot_wait_time_mstar_planning_inflation_'+str(self.inflation_level)+'_'+str(self.workspace_x)+'_'+str(total_robots)+'_'+str(datetime.now().month)+'.txt'
        f = open(file_name, 'a')
        num_iteration = int(sum(1 for line in open(file_name))/3)
        
        
        f.write('Mstar_robot_economy_waitTime_'+ str(num_iteration+1)+' = ' + repr(tmp_economy_wait_time) + '\n')
        
        f.write('Mstar_robot_regular_waitTime_'+ str(num_iteration+1)+' = ' + repr(tmp_regular_wait_time) + '\n') 
             
        f.write('Mstar_robot_premium_waitTime_'+ str(num_iteration+1)+' = ' + repr(tmp_premium_wait_time) + '\n')
        f.close()
        
        return (sum(tmp_wait_time_all) / total_robots)
       


def populateRobotInformationMstar(workspace_x, workspace_y,initpos,goals, neighborDirectedGraph, workspace_1, robot_arrival_time):
    
    robot_list = []
    category = 1
    
    for tmp_id in range(len(initpos)):
        
        tmp_robot_path = []
        tmp_init_pos = initpos[tmp_id]
        tmp_goal_pos = goals[tmp_id]
        tmp_arrival_time = robot_arrival_time[tmp_id]
        
        tmp_robot = MobileRobot(tmp_id, category, tmp_init_pos , tmp_goal_pos, tmp_robot_path, neighborDirectedGraph, workspace_x, workspace_y, workspace_1,tmp_arrival_time)
        robot_list.append(tmp_robot)
        print("Path of Robot: "+str(tmp_id)+ " Arival Time: " +str(tmp_arrival_time))
       
    return robot_list

    
def execute_Mstar_plan(workspace_x, workspace_y, init_pos, goal_pos, workspace_2, neighborDirectedGraph, workspace_1, robot_arrival_time, current_inflation_level):
   
    computation_time = 0
    max_path_length = 0
    avg_path_length = 0  
    print(robot_arrival_time)
    robot_list = populateRobotInformationMstar(workspace_x, workspace_y, init_pos, goal_pos,neighborDirectedGraph, workspace_1, robot_arrival_time)
    mstar_executor = Mstar_Plan_Executor(workspace_x, workspace_y, workspace_2, neighborDirectedGraph, workspace_1, robot_list, current_inflation_level) 
    mstar_executor.excute_path_Mstar()
    computation_time = sum(mstar_executor.plan_formulation_time)
    max_path_length = mstar_executor.max_plan_execution_time
    avg_path_length = mstar_executor.get_average_path_mstar()
    avg_wait_time = mstar_executor.save_wait_time()
    print(" Computation Time: "+str(computation_time)+ " Execution Time: "+str(max_path_length)  + " Avg Path Length " + str(avg_path_length))
    """max_path_length = excute_path_Mstar(robot_list)
    avg_path_length = get_average_path_Mstar(robot_list)
    print("Mstar:  Workspace: " + str(workspace_x) + " by " + str(workspace_y) +" Total Robots: "+ str(len(init_pos)), end = " ")
    print(" Plan Computation Time: " + "{0:.4f}".format(computation_time), end = " ")
    print(" Max Path Length: " + str(max_path_length) + " Average Path Length: " + str(avg_path_length))"""
    
    tmp_line_part_1 = "Mstar Inflation Level: " + str(mstar_executor.inflation_level) +  " Workspace " + str(workspace_x) + " by " + str(workspace_y) +" Total Robots: "+ str(len(init_pos)) 
    tmp_line_part_2 = " Planning Time: " +  "{0:.4f}".format(computation_time) 
    tmp_line_part_3 = " Max Path Length: " + str(max_path_length) + " Avg. Path Length: " + "{0:.4f}".format(avg_path_length)
    line_total =  tmp_line_part_1 + tmp_line_part_2 + tmp_line_part_3 + " \n"
    print(line_total) 
    file_name=open('result.txt','a')
    file_name.write(line_total)
    file_name.close()
    
    return  computation_time, max_path_length, avg_path_length, avg_wait_time            

    


    
