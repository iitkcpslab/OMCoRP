import sys
import random
import time
from datetime import datetime
import numpy as np
import networkx as nx
import pandas as pd
import gc
import itertools
import matplotlib.pyplot as plt
from colors import color
from mobile_robot import *
from config_values import *

#sys.stdout = open('output.txt','wt')
valid = []
matrix = []
FREE = 0
OBS = 1

   
# Generate shortest path for a robot
def shortest_path(workspace_x, workspace_y, init_pos,goal_pos, neighborDirectedGraph):
    
    start_index = init_pos[0] + init_pos[1] * workspace_x + 1 
    goal_index = goal_pos[0] + goal_pos[1] * workspace_x + 1
    
    tmp_path=nx.dijkstra_path(neighborDirectedGraph,start_index,goal_index)
    path = []
    for tmp_location in tmp_path:
        tmp_x = ((tmp_location - 1) % workspace_x)
        tmp_y = int((tmp_location - 1) / workspace_x)
        path.append([tmp_x,tmp_y])
    
    return path    


# Creates robot modules    
def populateRobotInoformationAuction(workspace_x, workspace_y, init_pos,goals, neighborDirectedGraph, workspace_1, robot_arrival_time):
    
    robot_list = []
    category = 1
    tmp_planning_time = []
    
    for tmp_id in range(len(init_pos)):
        tmp_init_pos = init_pos[tmp_id]
        tmp_goal_pos = goals[tmp_id]
        tmp_arrival_time = robot_arrival_time[tmp_id]
        start_time = time.time()
        tmp_robot_path = shortest_path(workspace_x, workspace_y, tmp_init_pos,tmp_goal_pos, neighborDirectedGraph)
        end_time = time.time()
        tmp_planning_time.append(end_time - start_time)
        tmp_robot = MobileRobot(tmp_id, category, tmp_init_pos, tmp_goal_pos, tmp_robot_path, neighborDirectedGraph, workspace_x, workspace_y, workspace_1, tmp_arrival_time)
        robot_list.append(tmp_robot) 
        
    planning_time = max(tmp_planning_time)
            
    return robot_list, planning_time
    
       

    
    
    
class Auction_Plan_Executor:
    def __init__(self, workspace_x, workspace_y, neighborDirectedGraph, workspace_1, robot_list):
        self.workspace_x =  workspace_x
        self.workspace_y =  workspace_y
        self.neighborDirected = neighborDirectedGraph.copy()
        self.workspace_directional = workspace_1
        self.robot_list = robot_list
        self.active_robot_list = []
        self.last_replan_time = -1.0
        self.newly_arrived_robots = []
        self.max_plan_execution_time = 0
        self.timeout_flag = 0
        self.syn_flag = flag_synchronous_transition
        self.vcg_resolving_time = []
        self.vcg_resolving_time_1 = []
        self.total_worst_case_scenarios = 1
        self.max_path_length_allowed = max_permissible_path
    
    
    def get_active_robot_list(self, current_time_step):
        self.active_robot_list.clear()
        for tmp_robot in self.robot_list:
            tmp_robot.show_status()
            if(tmp_robot.arrival_time <= current_time_step):
                self.active_robot_list.append(tmp_robot)
            
            
        print(" List of active robots: ........")
        for tmp_robot in self.active_robot_list:
            tmp_robot.show_status()        
        
        
        
    def check_collision(self):
        collision_location_dict = {}
        for tmp_robot in self.robot_list:
            tmp_location = tmp_robot.get_current_location()
            tmp_key = str(tmp_location[0]) + " " + str(tmp_location[1])
            collision_location_dict.setdefault(tmp_key,[])
            collision_location_dict[tmp_key].append(tmp_robot.id)
         
        
    def is_free_location_auction(self, location, robot_id):
        # Return 1 if the location is free 
        index = 0
        for tmp_robot in self.robot_list:
            index = tmp_robot.check_occupancy(location, robot_id)
            if(index == 1):
                return 0
             
        return 1
        
    
                    
                    
    def get_neighbor_list(self, loc_x, loc_y):
        neighbor_list = []
        tmp_next_location = []
        tmp_next_location.append([loc_x + 1, loc_y])
        tmp_next_location.append([loc_x - 1, loc_y])
        tmp_next_location.append([loc_x, loc_y + 1])
        tmp_next_location.append([loc_x, loc_y - 1])
        tmp_next_location.append([loc_x + 1, loc_y + 1])
        tmp_next_location.append([loc_x - 1, loc_y - 1])
        tmp_next_location.append([loc_x - 1, loc_y + 1])
        tmp_next_location.append([loc_x + 1, loc_y - 1])
        next_location = []
        tmp_workspace_x,tmp_workspace_y = np.array(self.workspace_directional).shape
         
        for tmp_location in tmp_next_location:
            tmp_x = tmp_location[0]
            tmp_y = tmp_location[1]
            if ( (0 <= tmp_x < tmp_workspace_x) and (0 <= tmp_y < tmp_workspace_y) and (self.workspace_directional[tmp_x][tmp_y] > 4) ):
                
                next_location.append([tmp_x,tmp_y])  
    
        for tmp_location in next_location:
            for tmp_robot in self.robot_list:
                if(tuple(tmp_location) == tuple(tmp_robot.current)):
                    neighbor_list.append(tmp_robot.id)
                   
        return neighbor_list, next_location 
    
        
    def get_average_path_auction(self):
        tmp_average_path = 0
        for tmp_robot in self.robot_list:
            tmp_average_path = tmp_average_path + tmp_robot.total_path_traversed
            total_robots = len(self.robot_list)
        return  (tmp_average_path/total_robots)
        
        
    def check_collision_config(self, robot_config):
        """
        get all the locations from a config and estimate the set of th elocations.
        if the cardinality of the set is less than the total locations, a collision 
        is detected.
        If collision detected, then return 1
        """
        robot_location_all = []
        for tmp_config in robot_config:
            tmp_location = tmp_config['location']
            robot_location_all.append(tmp_location)
            
        robot_location_set = list(set(robot_location_all))
        
        flag = 0
        if(len(robot_location_all) > len(robot_location_set)):
            flag = 1
            
        
        
        return flag
        
        
    def check_validity_interscetion_occupancy(self, intersection_key, robot_config):
        """
        return 1 if all the cells of any intersection are occupied
        """
        intersection_size = []
        intersection_size.append(1)
        total_robot_within_intersection = 0
        robot_location_all = []
        for tmp_config in robot_config:
            tmp_location = tmp_config['location']
            tmp_x = tmp_location[0]
            tmp_y = tmp_location[1]
            robot_location_all.append(tmp_location)
            
            if(self.workspace_directional[tmp_x][tmp_y] > 4):
                total_robot_within_intersection = total_robot_within_intersection + 1
                neighbor_robot_list, neighbor_intersection_location = self.get_neighbor_list(tmp_x, tmp_y)
                intersection_size.append(len(neighbor_intersection_location))
                
                
        flag_occupancy = 0
        
        cells_within_intesection =  intersection_key.split()
        total_cells_intersection = int(len(cells_within_intesection)/2)
             
        
        if(total_robot_within_intersection > total_cells_intersection - 1):
            flag_occupancy = 1
                        
        return flag_occupancy
        
    def get_robot_list_intersection(self, intersection_key, robot_all_vcg):
        
        splited_keys = intersection_key.split()
        intersection_cells = []
        neighbor_list = []
        neighbor_list_not_vcg = []
        neighbor_location = []
        for i  in range(0, len(splited_keys), 2):
            tmp_x = int(splited_keys[i])
            tmp_y = int(splited_keys[i + 1])
            intersection_cells.append([tmp_x, tmp_y])
            
        
        
        for tmp_location in intersection_cells:
            for tmp_robot in self.robot_list:
                if(tuple(tmp_location) == tuple(tmp_robot.current)):
                    neighbor_list.append(tmp_robot.id)
        
        

        for i in range(len(neighbor_list)):
                tmp_id = neighbor_list[i]
                flag_within_list = 0
                for j in range(len(robot_all_vcg)):
                    if(tmp_id == robot_all_vcg[j]):
                        flag_within_list = 1
                
                if(flag_within_list == 0):    
                    tmp_location = self.robot_list[tmp_id].get_current_location()
                    neighbor_list_not_vcg.append(tmp_id)
                    neighbor_location.append(tmp_location)        
        
        return neighbor_list_not_vcg, neighbor_location
        
    def get_all_valid_configuration(self, intersection_key, robot_info_all, robot_id_to_be_excluded):
        
         
        robot_config = []
        robot_id_all = []
        
        for tmp_robot_info in robot_info_all:
            tmp_robot_config = []
            tmp_robot_id = tmp_robot_info['id']
            tmp_robot_current_location = tmp_robot_info['location']
            tmp_robot_next_location = tmp_robot_info['target_location']
            tmp_robot_auctioned_bid = tmp_robot_info['auctioned_bid']
            
            robot_id_all.append(tmp_robot_id)
                
            tmp_robot_current_config = {'id': tmp_robot_id, 'location': tmp_robot_current_location, 'value': 0}
            tmp_robot_next_config = {'id': tmp_robot_id, 'location': tmp_robot_next_location, 'value': tmp_robot_auctioned_bid}
                
            tmp_robot_config.append(tmp_robot_current_config)
            tmp_robot_config.append(tmp_robot_next_config)
            
            robot_config.append(tmp_robot_config)
           
        
        if(len(robot_info_all) >= 1):
            
            neighbor_robots_intersection, neighbor_locations_intersection = self.get_robot_list_intersection(intersection_key, robot_id_all)
            
            
           
            for i in range(len(neighbor_robots_intersection)):
                tmp_id = neighbor_robots_intersection[i]
                if(tmp_id != robot_id_to_be_excluded):
                    tmp_robot_config = []     
                    tmp_robot_current_config = {'id': neighbor_robots_intersection[i], 'location': tuple(neighbor_locations_intersection[i]), 'value': 0}
                    tmp_robot_config.append(tmp_robot_current_config)
                    robot_config.append(tmp_robot_config)
                    
        
                
                
        all_config = itertools.product(*robot_config)
        
        
        valid_robot_config = []        
        
        for tmp_robot_config in all_config:
            
            
                
            flag_collision = self.check_collision_config(tmp_robot_config)
            flag_intersection_occupancy = self.check_validity_interscetion_occupancy(intersection_key, tmp_robot_config)
            if((flag_collision == 0) and (flag_intersection_occupancy == 0)):
                valid_robot_config.append(tmp_robot_config)
                
        return valid_robot_config
        
    def get_max_valued_valid_config(self, valid_configs):
        max_value = -1
        max_config = []
        
        
        for tmp_config in valid_configs:
            tmp_value = 0
            tmp_total_robot = 0
            for tmp_robot_info in tmp_config:
                tmp_value = tmp_value + tmp_robot_info['value']
                if(tmp_robot_info['value'] > 0):
                    tmp_total_robot = tmp_total_robot + 1
                    
               
            tmp_total_robot = 1
            if((tmp_value * tmp_total_robot) > max_value):
                max_value = tmp_value * tmp_total_robot 
                max_config.clear()
                for tmp_robot_info in tmp_config:
                    max_config.append(tmp_robot_info)
                    
        
        total_value = 0  
        tmp_max_robots = 0
        for tmp_robot_info in list(max_config):
               
            total_value = total_value + tmp_robot_info['value']
            tmp_max_robots = tmp_max_robots  + 1
        if(total_value == 0):
            self.total_worst_case_scenarios = self.total_worst_case_scenarios  + 1
            
        
        return max_config, total_value
        
    def get_max_bidders(self, max_valid_config):
       
        max_bidders = []
        for tmp_config in max_valid_config:
            if(tmp_config['value'] > 0):
                tmp_id = tmp_config['id']
                max_bidders.append(tmp_id)
                
        return max_bidders
        
        
    def resolve_vcg_mechanism(self, auction_robot_dict):
       
        merged_robot_dict = {}
        tmp_vcg_time = [0]
        for key, robot_ids in auction_robot_dict.items():
            t0 = time.time() 
            location = key.split()
            tmp_x = int(location[0])
            tmp_y = int(location[1])
           
            auctioned_value = []
            
            if(( self.workspace_directional[tmp_x][tmp_y] > 4)):
                tmp_key = ""
                
                neighbor_list, intersection_cells = self.get_neighbor_list(tmp_x, tmp_y)
                intersection_cells.append([tmp_x, tmp_y])
                for tmp_cell in sorted(intersection_cells):
                    
                    tmp_key =  tmp_key + str(tmp_cell[0]) + " " + str(tmp_cell[1]) + " "
                    
                    
                merged_robot_dict.setdefault(tmp_key,[]) 
            t1 = time.time()
            tmp_vcg_time.append(t1-t0)
            self.vcg_resolving_time.append((t1-t0))
            
          
        if(len(tmp_vcg_time) != 0):    
            tmp_max_time_1 = max(tmp_vcg_time)
            
        else:
            tmp_max_time_1 = 0
        tmp_vcg_time.clear()
        
        
        for key, robot_ids in auction_robot_dict.items():
            t0 = time.time()
            location = key.split()
            tmp_x = int(location[0])
            tmp_y = int(location[1])
           
            auctioned_value = []
            
            if(( self.workspace_directional[tmp_x][tmp_y] > 4)):
                tmp_key = ""
                
                neighbor_list, intersection_cells = self.get_neighbor_list(tmp_x, tmp_y)
                intersection_cells.append([tmp_x, tmp_y])
                for tmp_cell in sorted(intersection_cells):
                    
                    tmp_key =  tmp_key + str(tmp_cell[0]) + " " + str(tmp_cell[1]) + " "
                    
                    
                
                for value in robot_ids:
                    tmp_index = int(value)
                    tmp_robot = self.robot_list[tmp_index]
                    tmp_location = tmp_robot.get_current_location()
                    tmp_individual_auction_value = tmp_robot.get_auction_value()
                    auctioned_value.append(tmp_individual_auction_value)
                    tmp_robot_info= {'id': tmp_index, 'auctioned_bid': tmp_individual_auction_value, 'location': tuple(tmp_location), 'target_location' : tuple([tmp_x, tmp_y])}
                    merged_robot_dict[tmp_key].append(tmp_robot_info)
                
            t1 = time.time()
            tmp_vcg_time.append(t1-t0)
            self.vcg_resolving_time.append((t1-t0))
        
        if(len(tmp_vcg_time) != 0):    
            tmp_max_time_2 = max(tmp_vcg_time)
           
        else:
            tmp_max_time_2 = 0
        tmp_vcg_time.clear()    
           
               
        
        
        
        for key, robot_info_total in merged_robot_dict.items():
            
            t2 = time.time()                
            valid_configs = self.get_all_valid_configuration(key, robot_info_total, -1)
            
            max_valid_config, max_config_value = self.get_max_valued_valid_config(valid_configs)
            max_bidders = self.get_max_bidders(max_valid_config)
            
            payment_config = self.calculate_payment(key, max_valid_config, robot_info_total)
            
            for tmp_robot_info in robot_info_total:
                tmp_index =  tmp_robot_info['id'] 
                tmp_robot = self.robot_list[tmp_index]
                tmp_robot.resolve_vcg(max_bidders, payment_config)                   
            t3 = time.time()
            
            tmp_vcg_time.append(t3-t2)
           
           
        tmp_max_time_3  = 0.0
        if(len(tmp_vcg_time) != 0):
          
            tmp_max_time_3 = np.mean(np.array(tmp_vcg_time))
        
          
        
        if(tmp_max_time_3 <= 0.025):
            tmp_max_time_3 = 0.025
        self.vcg_resolving_time.append(tmp_max_time_3)
        self.vcg_resolving_time_1.append(tmp_max_time_1 + tmp_max_time_2 + tmp_max_time_3)
        
        
    #..................................................................................

    def calculate_payment(self, key, max_config, robot_info_total):
        
        total_value = 0
        payment_config = []
        
        for tmp_config in max_config:
               
                total_value = total_value + tmp_config['value']
                
        
        if(len(max_config) == 1):
            for tmp_config in max_config:
                tmp_index = tmp_config['id']
                tmp_value = tmp_config['value']
                tmp_robot_info= {'id': tmp_index, 'value': tmp_value, 'payment': 0}
                payment_config.append(tmp_robot_info)
        else:
         
            for tmp_config in max_config:
               
                tmp_id = tmp_config['id']
                tmp_value = tmp_config['value']
                if(tmp_value > 0):
                    
                    robot_info_partial = []
                    for tmp_robot_info in robot_info_total:
                        if(tmp_robot_info['id'] != tmp_id):
                            robot_info_partial.append(tmp_robot_info)
                        
                    valid_configs = self.get_all_valid_configuration(key, robot_info_partial, tmp_id)
                                       
                    max_valid_config, max_config_value_partial = self.get_max_valued_valid_config(valid_configs)
                    payment =  max_config_value_partial - (total_value - tmp_value)    
                    tmp_robot_info= {'id': tmp_id, 'value': tmp_value, 'payment': payment}
                    payment_config.append(tmp_robot_info)
                    
                else:
                    tmp_robot_info= {'id': tmp_id, 'value': 0, 'payment': 0}
                    payment_config.append(tmp_robot_info)
                   
        return payment_config
        
           
           
    #.....................................................................
    def excute_path_vcg(self):
        flag = 1
        gc.collect()          
        
        
        print("Showing Status of All Robots:.....")
        for tmp_robot in self.robot_list:
            tmp_robot.show_status()
        #input(" Enter a key....")    
        while(flag == 1 and self.max_plan_execution_time < self.max_path_length_allowed):
            print("Round ...>>>>>>>>>>>>>>>>>>>>>>>>>       " + str(self.max_plan_execution_time))
           
            flag = 0
            auction_robot_dict = {}
            self.check_collision()
            self.get_active_robot_list(self.max_plan_execution_time)
            for tmp_robot in self.active_robot_list:
                robot_next_location =tmp_robot.get_next_location()
                      
                if robot_next_location != [-1, -1]:
                    flag = 1
                    occupancy_flag = self.is_free_location_auction(robot_next_location,tmp_robot.id )
                   
                    tmp_x = robot_next_location[0]
                    tmp_y = robot_next_location[1]
                    if(occupancy_flag == 1 or (self.workspace_directional[tmp_x][tmp_y] > 4)):
                      
                        tmp_robot.status = 'M'
                        tmp_key = str(robot_next_location[0]) + " " + str(robot_next_location[1])
                        auction_robot_dict.setdefault(tmp_key,[])
                        auction_robot_dict[tmp_key].append(tmp_robot.id)
                    else:    
                        tmp_robot.status = 'W'
                  
           
            self.resolve_vcg_mechanism(auction_robot_dict) 
            
           
           
            
            if(self.syn_flag == 1):
                for tmp_robot in self.active_robot_list:
                    tmp_robot_id = tmp_robot.id
                    tmp_location = tmp_robot.get_next_location()
                    
                  
                    if((tmp_location[0] != -1) and (tmp_location[1]!= -1)):
                        tmp_status = self.updateStatusByNextLocation(tmp_robot_id)
                      
                        tmp_robot.status = tmp_status
            
            for tmp_robot in self.active_robot_list:
                tmp_robot.update_location()
        
                  
            self.max_plan_execution_time = self.max_plan_execution_time + 1
            
        return self.max_plan_execution_time 
        
        
    def updateStatusByNextLocation(self, robot_id):
        tmp_status = "W"
        next_robot_id = -1
        if(self.robot_list[robot_id].status == "M"):
            tmp_status = "M"
            return tmp_status
        tmp_location = self.robot_list[robot_id].get_next_location()
        if(self.workspace_directional[tmp_location[0]][tmp_location[1]] > 4):
            tmp_status = self.robot_list[robot_id].status
            return tmp_status
            
        for tmp_robot in self.active_robot_list:
            tmp_flag = tmp_robot.check_occupancy(tmp_location, robot_id)
            if(tmp_flag == 1):
                next_robot_id = tmp_robot.id
       
        if(next_robot_id != -1):
            tmp_status = self.updateStatusByNextLocation(next_robot_id)
        else:
            tmp_status = self.robot_list[robot_id].status
        
                
        return tmp_status
        
    def calcualte_and_show_payments(self):
        tmp_values = []
        tmp_payments = []
        tmp_total_values = []
        economy_payment = []
        regular_payment = []
        premium_payment = [] 
        
        economy_total_value = []
        regular_total_value = []
        premium_total_value = [] 
        
        for tmp_robot in self.robot_list:
            tmp_robot_value = sum(tmp_robot.bid_values)
            tmp_robot_payment = sum(tmp_robot.payment_values)
            tmp_robot_total_value = tmp_robot_value - tmp_robot_payment
            
            tmp_values.append(float(format(tmp_robot_value, '0.4f')))
            tmp_payments.append(float(format(tmp_robot_payment, '0.4f')))
            tmp_total_values.append(float(format(tmp_robot_total_value, '0.4f')))
            
            
            # Saving the payment according to class of the robot
            tmp_category = tmp_robot.category
            
            if(tmp_category == "economy"):
                economy_payment.append(tmp_robot_payment) 
                economy_total_value.append(tmp_robot_total_value)   
                
            if(tmp_category == "regular"):
                regular_payment.append(tmp_robot_payment)
                regular_total_value.append(tmp_robot_total_value)
                
            if(tmp_category == "premium"):
                premium_payment.append(tmp_robot_payment)
                premium_total_value.append(tmp_robot_total_value)
            
        total_robots = len(self.robot_list)
        file_name = 'robot_auction_'+str(self.workspace_x)+'_'+str(total_robots)+'.txt'
        f = open(file_name, 'a')
        num_iteration = int(sum(1 for line in open(file_name))/3)
        
        
        
        f.write('Auction_robot_values_'+ str(num_iteration+1)+' = ' + repr(tmp_values) + '\n') 
        f.write('Auction_robot_payments_'+ str(num_iteration+1)+' = ' + repr(tmp_payments) + '\n') 
        f.write('Auction_robot_total_values_'+ str(num_iteration+1)+' = ' + repr(tmp_total_values) + '\n')
        f.close()
        
        
        file_name = 'robot_auction_'+str(self.workspace_x)+'.txt'
        f = open(file_name, 'a')
        num_iteration = int(sum(1 for line in open(file_name))/3)
                
        f.write('Auction_robot_values_'+ str(num_iteration+1)+' = ' + repr(tmp_values) + '\n') 
        f.write('Auction_robot_payments_'+ str(num_iteration+1)+' = ' + repr(tmp_payments) + '\n') 
        f.write('Auction_robot_total_values_'+ str(num_iteration+1)+' = ' + repr(tmp_total_values) + '\n')
        f.close()
        
        file_name = 'robot_payment_'+str(self.workspace_x)+'_'+str(total_robots)+'.txt'
        f = open(file_name, 'a')
        num_iteration = int(sum(1 for line in open(file_name))/3)
               
        f.write('Economy_robot_payment_'+ str(num_iteration+1)+' = ' + repr(economy_payment) + '\n') 
        f.write('Regular_robot_payment_'+ str(num_iteration+1)+' = ' + repr(regular_payment) + '\n') 
        f.write('Premium_robot_payment_'+ str(num_iteration+1)+' = ' + repr(premium_payment) + '\n')
        f.close()
        
        file_name = 'robot_total_value_'+str(self.workspace_x)+'_'+str(total_robots)+'.txt'
        f = open(file_name, 'a')
        num_iteration = int(sum(1 for line in open(file_name))/3)
               
        f.write('Economy_robot_total_value_'+ str(num_iteration+1)+' = ' + repr(economy_total_value) + '\n') 
        f.write('Regular_robot_total_value_'+ str(num_iteration+1)+' = ' + repr(regular_total_value) + '\n') 
        f.write('Premium_robot_total_value_'+ str(num_iteration+1)+' = ' + repr(premium_total_value) + '\n')
        f.close()
        
    
    def save_wait_time(self):
        
        
        tmp_wait_time_all = []
        
        tmp_economy_wait_time = []
        tmp_regular_wait_time = []
        tmp_premium_wait_time = []
        
        tmp_economy_auction_participation = []
        tmp_regular_auction_participation = []
        tmp_premium_auction_participation = []
        
        for tmp_robot in self.robot_list:
            
            tmp_wait_time_all.append(tmp_robot.waitTime)
            tmp_category = tmp_robot.category
            
            if(tmp_category == "economy"):
                
                tmp_economy_wait_time.append(tmp_robot.waitTime)
                tmp_economy_auction_participation.append(tmp_robot.totalAuctionParticipation)
                
            if(tmp_category == "regular"):
               
                tmp_regular_wait_time.append(tmp_robot.waitTime)
                tmp_regular_auction_participation.append(tmp_robot.totalAuctionParticipation)
                
            if(tmp_category == "premium"):
               
                tmp_premium_wait_time.append(tmp_robot.waitTime)
                tmp_premium_auction_participation.append(tmp_robot.totalAuctionParticipation)
                
        total_robots = len(self.robot_list)
        file_name = 'robot_wait_time_auction_planning_'+str(self.workspace_x)+'_'+str(total_robots)+'_'+str(datetime.now().month)+'.txt'
        f = open(file_name, 'a')
        num_iteration = int(sum(1 for line in open(file_name))/6)
        
        f.write('Auction_robot_economy_totalAuctionParticipation_'+ str(num_iteration+1)+' = ' + repr(tmp_economy_auction_participation) + '\n')
        f.write('Auction_robot_economy_waitTime_'+ str(num_iteration+1)+' = ' + repr(tmp_economy_wait_time) + '\n')
        
        f.write('Auction_robot_regular_totalAuctionParticipation_'+ str(num_iteration+1)+' = ' + repr(tmp_regular_auction_participation) + '\n')
        f.write('Auction_robot_regular_waitTime_'+ str(num_iteration+1)+' = ' + repr(tmp_regular_wait_time) + '\n') 
        
        f.write('Auction_robot_premium_totalAuctionParticipation_'+ str(num_iteration+1)+' = ' + repr(tmp_premium_auction_participation) + '\n')
        f.write('Auction_robot_premium_waitTime_'+ str(num_iteration+1)+' = ' + repr(tmp_premium_wait_time) + '\n')
        f.close()
        
        
        return (sum(tmp_wait_time_all) / total_robots)
       
       
     
        
    
    
        
        
        
    
   
           
    
def execute_auction_plan(workspace_x, workspace_y, init_pos, goal_pos, neighborDirectedGraph, workspace_1, robot_arrival_time):
    
    robot_list, planning_time = populateRobotInoformationAuction(workspace_x, workspace_y, init_pos, goal_pos, neighborDirectedGraph, workspace_1, robot_arrival_time)
   
      
    vcg_executor = Auction_Plan_Executor(workspace_x, workspace_y, neighborDirectedGraph, workspace_1, robot_list)
    max_path_length = vcg_executor.excute_path_vcg()
    avg_path_length = vcg_executor.get_average_path_auction()
    total_worst_scenarios = 1 * vcg_executor.total_worst_case_scenarios 
    #vcg_time = ((sum(vcg_executor.vcg_resolving_time))/total_worst_scenarios) * max_path_length # 
    vcg_time = (sum(vcg_executor.vcg_resolving_time))
    vcg_time_1 = sum(vcg_executor.vcg_resolving_time_1)
    total_vcg = vcg_time_1 * max_path_length 
    vcg_executor.calcualte_and_show_payments()
    avg_wait_time = vcg_executor.save_wait_time()
    
    print("Auction:  Workspace: " + str(workspace_x) + " by " + str(workspace_y) +" Total Robots: "+ str(len(init_pos)), end = " ")
    print(" Plan Computation Time: " + "{0:.4f}".format(planning_time), end = " ")
    print(" Max Path Length: " + str(max_path_length) + " Average Path Length: " + str(avg_path_length), end = " ")
    print(" Total Worst Cases: " + "{0:.4f}".format(total_worst_scenarios) + " VCG Time: " + "{0:.4f}".format(vcg_time) + " VCG Time 1: " + "{0:.4f}".format(vcg_time_1))
   
    tmp_line_part_1 = "Auction Workspace " + str(workspace_x) + " by " + str(workspace_y) +" Total Robots: "+ str(len(init_pos)) 
    tmp_line_part_2 = " Planning Time: " +  "{0:.4f}".format(planning_time) 
    tmp_line_part_3 = " Max Path Length: " + str(max_path_length) + " Avg. Path Length: " + "{0:.4f}".format(avg_path_length)
    line_total =  tmp_line_part_1 + tmp_line_part_2 + tmp_line_part_3+"\n"
     
    file_name=open('result.txt','a')
    file_name.write(line_total)
    file_name.close()            
    return planning_time, max_path_length, avg_path_length, vcg_time, total_worst_scenarios, avg_wait_time                       	        
               
  

    


