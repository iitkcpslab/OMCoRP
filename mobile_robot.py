import random
import numpy as np
import networkx as nx
class MobileRobot():

    def __init__(self,tmp_id, category, source, target, tmp_path, neighborGraph, workspace_x, workspace_y, workspace_1, arrival_time):
        self.id = tmp_id
        self.category=category
        self.source=source
        self.target=target
        self.current = source
        self.workspace_x = workspace_x
        self.workspace_y = workspace_y
        self.path=tmp_path
        self.pos=0
        self.bid=0
        self.bid_values = []
        self.payment_values = []
        self.f=0
        self.status = 'S'
        self.waitTime = 0
        self.waitVCGTime = 0
        self.totalAuctionParticipation = 0
        #self.neighborDirectedGraph = neighborGraph.copy()
        #self.workspace = workspace_1
        self.arrival_time = arrival_time
        self.total_path_traversed = 0
        self.priority=len(self.path)*0.04 
        tmp_random_value = random.uniform(0, 1)
        """
        if(tmp_random_value <= 0.4):
            self.category = "economy"
        elif(tmp_random_value > 0.4 and tmp_random_value <= 0.8):
            self.category = "regular"
        else:
            self.category = "premium"
        """
        tmp_class = tmp_id%3  # 1/3 rd economy, 1/3 rd regular, 1/3 premium
        if(tmp_class == 0):
            self.category = "economy"
            self.waitTime_weight = 0.4
        if(tmp_class == 1):
            self.category = "regular"
            self.waitTime_weight = 0.65
        if(tmp_class == 2):
            self.category = "premium"
            self.waitTime_weight = 1.0
        
    def get_next_location(self):
        if(tuple(self.current) != tuple(self.target)):
            next_location = self.path[self.pos+1]
        else:
            next_location = [-1, -1]
            
        return next_location
        
    def check_occupancy(self, location, robot_id):
        #print("Robot: "+str(self.id)+ " Current Location: "+str(list(self.current))+ " Location: "+ str(list(location)))
        if(list(self.current) == list(location) and (self.id != robot_id)):
        
             
            return 1
        else:
            
            return 0
            
    def get_current_location(self):
        return self.current

    def get_goal_location(self):
        return self.target 
        
    def update_waiting_time(self):
        robot_next_location =self.get_next_location()
        if tuple(robot_next_location) != tuple([-1, -1]):
            robot_current_location =self.get_current_location()
            if(tuple(robot_next_location) == tuple(robot_current_location)):
                self.waitTime = self.waitTime + 1
            
    def update_location(self):
        
        if(self.status == 'M'):
            tmp_index = self.pos + 1
            next_location = self.path[tmp_index]
            self.current = list(next_location)
            self.pos = self.pos + 1
            
        if(self.status == 'W'):
            self.waitTime = self.waitTime + 1
            
        next_location  = self.get_next_location()
        if(tuple(next_location) != tuple([-1,-1])):
            self.total_path_traversed = self.total_path_traversed + 1        
        self.status = 'S'
        print("Robot: "+str(self.id)+ " Location: " + str(self.current) + " Next Location:" + str(self.get_next_location()) + " Target: " +str(self.target) +" pos: " + str(self.pos) + " Status: " + str(self.status))
        
    def get_auction_value(self):
        
        class_weight = 0.05
        if(self.category == "regular"):
            class_weight = 0.1
        if(self.category == "premium" ):
            class_weight = 0.2 
        
        self.bid = (self.waitTime + 1) * class_weight * self.waitTime_weight # updated after 11/05/2019    
        
        
        self.totalAuctionParticipation = self.totalAuctionParticipation  + 1 
                
        
        return self.bid
        
        

        
    def resolve_auction(self, bidding_values, robot_ids, neighbor_list):
        max_bidding = max(bidding_values)
        tmp_previous_status = self.status
        [tmp_x, tmp_y] = self.get_current_location()
        if(len(neighbor_list) > 0 and self.workspace[tmp_x][tmp_y] <= 4):
            self.status = 'W'
           
        if(len(neighbor_list) < 1):      
            if(self.bid != max_bidding):
                self.status = 'W'
            else:
                max_index = bidding_values.index(max_bidding)
                max_bidder = robot_ids[max_index] 
                if(self.id != max_bidder):
                    self.status = 'W'
        
        # ... to avoid dead lock: checking the occupancy of auctioned location; if neighbor occupancy is greater than 2, all the bidder robots will wait at 
        # their current location
                          
        print("Robot id:" + str(self.id) + " Bidding: "+str(self.bid)+ " Status: " + str(self.status) + " Max bid: " + str(max_bidding) + " Previous status: " + str(tmp_previous_status))

    def resolve_vcg(self, max_bidder_ids, payment_config):
        flag_max_bidder = 0
        for i in range(len(max_bidder_ids)):
            if(self.id == max_bidder_ids[i]):
                flag_max_bidder = 1
                
        for tmp_payment in payment_config:
            if(self.id == tmp_payment['id']):
                self.bid_values.append(tmp_payment['value'])
                self.payment_values.append(tmp_payment['payment'])
                
        if(flag_max_bidder == 1):
            self.status = 'M'
        else:
            self.status = 'W'
            
        if(self.status == 'W'):
            self.waitVCGTime = self.waitVCGTime + 1
        


    def get_priority(self):
        return self.priority
        
    def resolve_priority_planning(self, location, priority):
       
        if(self.priority < priority):
            
            if(self.current != self.target):
                print("Robot: "+str(self.id) + " Priority: " + str(self.priority) + " Resolved Prioity: " + str(priority))
                reestimated_path = self.reestimate_shortest_path(location)
                print("Robot Id: "+str(self.id) + " Current Location: " + str(self.current))
                
            
    
        
     
    def show_status(self):
        print(" Robot id: " + str(self.id) + " Current Location: "+ str(self.current) + " Next Locatin: " + str(self.get_next_location()) +  " Goal Location: "+str(self.target) + " Arrival Time: " + str(self.arrival_time))
    

    def update_path_information(self, path):
        self.path=path
        self.pos=0
        print(" Robot: "+ str(self.id)+ " have changed its path information")
        

                   
               
        
        
                   
                
                        
