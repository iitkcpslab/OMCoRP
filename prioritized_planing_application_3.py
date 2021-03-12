import sys
import od_mstar
import time
import random
import timeout_decorator
import math

#import workspace_graph
from col_set_addition import OutOfTimeError, NoSolutionError
from datetime import datetime
import numpy as np
import networkx as nx
import pandas as pd
import matplotlib.pyplot as plt
from colors import color
from mobile_robot import *
from config_values import *


valid = []
matrix = []
FREE = 0
OBS = 1


class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.time_index = 0
        self.closed = 0

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
        
        

 
class PrioritizedPlanner_Plan_Executor:
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
        self.timed_directed_graph = self.generate_timed_directed_graph(self.neighborDirected)
        self.max_path_length_allowed = max_permissible_path * len(self.robot_list)/2 * (1 + flag_asynchronous_planning)
        
        
   

    def generate_timed_directed_graph(self, neighborDirectedGraph):
       
        timed_directed_graph = {}
        for tmp_node in neighborDirectedGraph:
            timed_directed_graph[tmp_node] = 0
            
             
        return timed_directed_graph
        
    def update_timed_directed_graph(self, node, time_index):
        tmp_index = self.timed_directed_graph[node]
        if(time_index >= tmp_index):
            self.timed_directed_graph[node] = time_index
            
        
        
    def generate_path(self):
        
        robot_path_distance = []
        for tmp_robot in ((self.newly_arrived_robots)):
            tmp_init_pos = self.robot_list[tmp_robot].get_current_location()
            tmp_goal_pos = self.robot_list[tmp_robot].get_goal_location()
            tmp_distance = abs(tmp_init_pos[0] - tmp_goal_pos[0]) + abs(tmp_init_pos[1] - tmp_goal_pos[1])
            tmp_category = self.robot_list[tmp_robot].category
            tmp_config = {'id': tmp_robot, 'distance': tmp_distance, 'category': tmp_category}
            robot_path_distance.append(tmp_config)
        
        
        
        robot_list_distance_sorted = sorted(robot_path_distance, key = lambda i: i['distance'])
        
        robot_list_max_distnce = robot_list_distance_sorted[::-1]
        for tmp_robot_info in robot_list_max_distnce:
            print("Id: " +str(tmp_robot_info['id']) + " Approx Distance: " +str(tmp_robot_info['distance']) + " Category: " +str(tmp_robot_info['category']))
        
        
        category_list = ["premium", "regular", "economy"]
        
        for tmp_category in category_list:
            print("Current Category: " +str(tmp_category))
            for tmp_robot_info in robot_list_max_distnce:
                if(tmp_category == tmp_robot_info['category']):
                    print("Id: " +str(tmp_robot_info['id']) + " Approx Distance: " +str(tmp_robot_info['distance']) + " Category: " +str(tmp_robot_info['category']))
                    tmp_robot_id = tmp_robot_info['id']
                    tmp_init_pos = self.robot_list[tmp_robot_id].get_current_location()
                    tmp_goal_pos = self.robot_list[tmp_robot_id].get_goal_location()
                    self.robot_list[tmp_robot_id].path = self.shortest_path(tmp_init_pos, tmp_goal_pos)
            
        
        self.active_robot_list.clear()
        for tmp_robot in self.robot_list:
            if(tmp_robot.arrival_time <= self.max_plan_execution_time):
                tmp_current_pos = tmp_robot.get_current_location()
                tmp_goal_pos = tmp_robot.get_goal_location()
                if(tuple(tmp_goal_pos) != tuple(tmp_current_pos)):
                    self.active_robot_list.append(tmp_robot)
        
            
            
    
    
    def get_max_index_timed_directed_graph(self, neighbor_node):
        
        tmp_index = self.timed_directed_graph[neighbor_node]
        
        return tmp_index          
    
    def shortest_path(self,init_pos,goal_pos):
    
        start_index = init_pos[0] + init_pos[1] *self.workspace_x + 1 
        goal_index = goal_pos[0] + goal_pos[1] * self.workspace_x + 1
        print("Initial Position: " +str(init_pos) + " Initial Index: " +str(start_index) + " Goal Location: " + str(goal_pos)+ " Goal index: " + str(goal_index))
    
        tmp_path=self.astar(start_index,goal_index)
        path = []
        for tmp_location in tmp_path:
            tmp_x = ((tmp_location - 1) % self.workspace_x)
            tmp_y = int((tmp_location - 1) / self.workspace_x)
            path.append([tmp_x,tmp_y])
        print(path)
       
        return path
        
        
        
    def astar(self,start, end):
        """Returns a list of tuples as a path from the given start to the given end in the given maze"""

        # Create start and end node
        start_node = Node(None, start)
        start_node.g = start_node.h = start_node.f =  start_node.time_index = 0
        end_node = Node(None, end)
        end_node.g = end_node.h = end_node.f = 0
        
        start_x = ((start - 1) % self.workspace_x)
        start_y = int((start - 1) / self.workspace_x)
        
        end_x = ((end - 1) % self.workspace_x)
        end_y = int((end - 1) / self.workspace_x)

        start_node.h = start_node.f = abs(start_x - end_x)  + abs(start_y - end_y)
        # Initialize both open and closed list
        open_list = []
        closed_list = []
        
        # Add the start node
        open_list.append(start_node)

        # Loop until you find the end
        while len(open_list) > 0:

            # Get the current node
            current_node = open_list[0]
            current_index = 0
        
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index
                            
            
            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            current_node.closed = 1
            closed_list.append(current_node)
            

            # Found the goal
            if current_node == end_node:
                
                path = []
                current = current_node
                parent_index = current_node.time_index + 1
                while current is not None:
                   
                    current_index = current.time_index
                    
                    while(current_index < parent_index):
                        path.append(current.position)
                        self.update_timed_directed_graph(current.position, current_index)
                        current_index = current_index + 1
                      
                    parent_index = current.time_index
                    current = current.parent
                
               
                return path[::-1] # Return reversed path

            
            # Generate children
            children = []
            
            tmp_successors = self.neighborDirected.neighbors(current_node.position)
            tmp_time_index = current_node.time_index + 1
            for tmp_node in tmp_successors:
                               
                max_index = self.get_max_index_timed_directed_graph(tmp_node)
                
                
                
                tmp_child_x = ((tmp_node - 1) % self.workspace_x)
                tmp_child_y = int((tmp_node - 1) / self.workspace_x)
                tmp_val  = self.workspace_directional[tmp_child_x][tmp_child_y]
                
                tmp_flag = 1
                if(tmp_node == start):
                    tmp_flag = 0

                if(tmp_node == end):
                    tmp_flag = 0
                    
                if(tmp_val > 0):
                    tmp_flag = 0
                
                if(tmp_flag == 0):
                    new_node = Node(current_node, tmp_node)
                    if(max_index >= tmp_time_index):
                        new_node.time_index = max_index + 1
                    else:
                        new_node.time_index = tmp_time_index
                    children.append(new_node)
            
            

            # Loop through children
            for child in children:

                # Child is on the closed list
                tmp_flag = 0 
                for closed_child in closed_list:
                    #if((child.position == closed_child.position) and (child.time_index >= closed_child.time_index)):
                    if((child.position == closed_child.position)):
                        tmp_flag = 1
                        continue
            
            
            
                
                if(tmp_flag == 0):
                    child.g = child.time_index + 1

                    tmp_h = 0
                    if(child.position == current_node.position):
                        tmp_h = 200
                    
                    tmp_child_x = ((child.position - 1) % self.workspace_x)
                    tmp_child_y = int((child.position - 1) / self.workspace_x)
                    tmp_val  = self.workspace_directional[tmp_child_x][tmp_child_y]
                    
                    if(tmp_val == 0 and child.position != end):
                        tmp_h = 200 
                    
                   
                    
                    tmp_child_x = ((child.position - 1) % self.workspace_x)
                    tmp_child_y = int((child.position - 1) / self.workspace_x)
                    tmp_end_x = ((end_node.position - 1) % self.workspace_x)
                    tmp_end_y = int((end_node.position - 1) / self.workspace_x)
                                
                    child.h = (abs(tmp_child_x - tmp_end_x) +  abs(tmp_child_y - tmp_end_y)) + tmp_h
                    
                    
                    child.f = child.g + child.h
            

               
                    tmp_flag_1 = 0                
                    for open_node in open_list:
                        if child.g >= open_node.g and child.position == open_node.position :
                            
                            tmp_flag_1 = 1
                            continue
                
                
                    if(tmp_flag_1 == 0):
                        open_list.append(child)
          
    

    def check_collision(self):
        collision_location_dict = {}
        for tmp_robot in self.robot_list:
            tmp_location = tmp_robot.get_current_location()
            tmp_key = str(tmp_location[0]) + " " + str(tmp_location[1])
            collision_location_dict.setdefault(tmp_key,[])
            collision_location_dict[tmp_key].append(tmp_robot.id)
        for key, robot_ids in collision_location_dict.items():
           
            if(len(robot_ids) > 1):
                print("Detect Collision:................................................. Robots: "+str(robot_ids) + " Location: " + str(key))
                                 
            
        
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
                
    def excute_path_Prioritized(self):
        
        flag = 1
        while(flag == 1 and self.max_plan_execution_time <= self.max_path_length_allowed):
            flag = 0
            self.check_collision()
            new_arrival_flag = self.check_new_arrivals()
            if(new_arrival_flag == 1):
                t0 = time.time()
                self.generate_path()
                t1= time.time()
                self.plan_formulation_time.append(t1-t0)
                
            
            for tmp_robot in self.active_robot_list:
                robot_next_location =tmp_robot.get_next_location()
                
                if tuple(robot_next_location) != tuple([-1, -1]):
                    flag = 1
                    tmp_robot.status = 'M'
                  
                        
            for tmp_robot in self.active_robot_list:
                tmp_robot.update_waiting_time() # Only for M* and PP Planning
                tmp_robot.update_location()
        
                  
            self.max_plan_execution_time = self.max_plan_execution_time + 1
            
        return self.max_plan_execution_time
        
        
    def get_average_path_prioritized(self):
        tmp_average_path = 0
        total_robots = len(self.robot_list)
        for tmp_robot in self.robot_list:
            tmp_average_path = tmp_average_path + tmp_robot.total_path_traversed
            
        return  (tmp_average_path/total_robots)
        
        
    def save_wait_time(self):
        
        
        tmp_wait_time_all = []        
        tmp_economy_wait_time = []
        tmp_regular_wait_time = []
        tmp_premium_wait_time = []
        
        
        for tmp_robot in self.robot_list:
            
            tmp_wait_time_all.append(tmp_robot.waitTime)
            tmp_category = tmp_robot.category
            
            if(tmp_category == "economy"):
                tmp_economy_wait_time.append(tmp_robot.waitTime)
                
                
            if(tmp_category == "regular"):
                tmp_regular_wait_time.append(tmp_robot.waitTime)
                
                
            if(tmp_category == "premium"):
                tmp_premium_wait_time.append(tmp_robot.waitTime)
                
                
        total_robots = len(self.robot_list)
        file_name = 'robot_wait_time_prioritized_planning_'+str(self.workspace_x)+'_'+str(total_robots)+'_'+str(datetime.now().month)+'.txt'
        f = open(file_name, 'a')
        num_iteration = int(sum(1 for line in open(file_name))/3)
        
        
        f.write('Prioritized_robot_economy_waitTime_'+ str(num_iteration+1)+' = ' + repr(tmp_economy_wait_time) + '\n')
        
        f.write('Prioritized_robot_regular_waitTime_'+ str(num_iteration+1)+' = ' + repr(tmp_regular_wait_time) + '\n') 
             
        f.write('Prioritized_robot_premium_waitTime_'+ str(num_iteration+1)+' = ' + repr(tmp_premium_wait_time) + '\n')
        f.close()
        
        return (sum(tmp_wait_time_all) / total_robots)
            
     


def populateRobotInformationPrioritizedPlanner(workspace_x, workspace_y,initpos,goals, neighborDirectedGraph, workspace_1, robot_arrival_time):
    
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

    
def execute_Prioritized_plan(workspace_x, workspace_y, init_pos, goal_pos, workspace_2, neighborDirectedGraph, workspace_1, robot_arrival_time, current_inflation_level):
    
    planning_time = 0
    max_path_length = 0
    avg_path_length = 0  
    print(robot_arrival_time)
    robot_list = populateRobotInformationPrioritizedPlanner(workspace_x, workspace_y, init_pos, goal_pos,neighborDirectedGraph, workspace_1, robot_arrival_time)
   
    prioritized_executor = PrioritizedPlanner_Plan_Executor(workspace_x, workspace_y, workspace_2, neighborDirectedGraph, workspace_1, robot_list, current_inflation_level)
   
    max_path_length = prioritized_executor.excute_path_Prioritized()
    planning_time = sum(prioritized_executor.plan_formulation_time) 
    avg_path_length =   prioritized_executor.get_average_path_prioritized()
    avg_wait_time = prioritized_executor.save_wait_time()
    
    
    print("Planning Time; " +str(planning_time) + " Max Path Length: " + str(max_path_length) + " Average Path length: " +str(avg_path_length), " Avg wait time: " +str(avg_wait_time))   
    return  planning_time, max_path_length, avg_path_length, avg_wait_time            

    


    
