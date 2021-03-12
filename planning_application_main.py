import sys
from joblib import Parallel, delayed
import multiprocessing
import gc
import random
from col_set_addition import OutOfTimeError, NoSolutionError
import time

import numpy as np
import networkx as nx
import pandas as pd
import matplotlib.pyplot as plt
from colors import color
from generate_workspace import *
from mstar_application2 import execute_Mstar_plan
from auction_application import execute_auction_plan
from prioritized_planing_application_3 import execute_Prioritized_plan
from config_values import *


import scipy.stats as st

import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.base import MIMEBase
from email import encoders





    

def generateWorkspace(workspace_x, workspace_y):
    
    
    tmp_workspace_2=[[1]*workspace_y for i in range(workspace_x)]
    
    
    tmp_workspace_1, valid_init_positions, valid_goal_positions = getWorkspace(workspace_x, workspace_y)
    neighborDirectedGraph = getNeighborGraph(workspace_x, workspace_y, tmp_workspace_1)
    
    print("..................Tmp_workspace_1.................") 
    for j in range(workspace_y-1,-1,-1):
         for i in range(0,workspace_x,1):
              print(str(tmp_workspace_1[i][j])+" ", end = "")
         print("")

    for j in range(workspace_y-1,-1,-1):
         for i in range(0,workspace_x,1): 
              if(tmp_workspace_1[i][j] >= 1):
                  tmp_workspace_2[i][j] = 0
    
    print("..................Tmp_workspace_2.................") 
    for j in range(workspace_y-1,-1,-1):
         for i in range(0,workspace_x,1):
             print(str(tmp_workspace_2[i][j])+" ", end = "")
         print("")
        
         
    return valid_goal_positions, tmp_workspace_2, tmp_workspace_1, neighborDirectedGraph
    
    
def execute_path_planning(workspace_x, workspace_y, robots, tmp_workspace_2, valid_goal_positions, neighborDirectedGraph, workspace_1):
   
    tmp_pos = random.sample(valid_goal_positions, 2*robots)
    init_pos = []
    goal_pos = []
    for i in range(robots):
        init_pos.append(tmp_pos[2*i])
        goal_pos.append(tmp_pos[2*i+1])    
    
    
    tmp_robot_number = len(init_pos)
    # arrival times of half of the robots are zero, rest are randomly slelected from the range 1 and workspace_x/2
    robot_arrival_time = [0]*tmp_robot_number
    if(flag_asynchronous_planning == 1):
        for i in range(int(tmp_robot_number/2), tmp_robot_number):
            robot_arrival_time[i] = random.randint(1, int(workspace_x * 2))
        
    print(" Initial Positions: " +str(init_pos))
    print("Goal Positions: " + str(goal_pos))
    print(robot_arrival_time)
    
    gc.collect()
    print("Starting Mstar Planning...")
    current_inflation_level = 1.0
    planning_time_mstar_1, plan_execution_time_mstar_1, avg_path_mstar_1, avg_wait_time_mstar_1 = execute_Mstar_plan(workspace_x, workspace_y, init_pos, goal_pos, tmp_workspace_2, neighborDirectedGraph, workspace_1, robot_arrival_time, current_inflation_level)
    
    gc.collect()
    current_inflation_level = 2.0
    planning_time_mstar_2, plan_execution_time_mstar_2, avg_path_mstar_2, avg_wait_time_mstar_2 = execute_Mstar_plan(workspace_x, workspace_y, init_pos, goal_pos, tmp_workspace_2, neighborDirectedGraph, workspace_1, robot_arrival_time, current_inflation_level)
    
    gc.collect()    
    print("Starting Auction Planning...")
    planning_time_auction, plan_execution_time_auction, avg_path_auction, vcg_resolving_time, total_worst_scenarios, avg_wait_time_auction  = execute_auction_plan(workspace_x, workspace_y, init_pos, goal_pos, neighborDirectedGraph, workspace_1, robot_arrival_time)
    
    gc.collect()    
    print("Starting Prioritized Planning...")
    current_inflation_level = 1.0
    planning_time_prioritized, max_path_prioritized, avg_path_prioritized, avg_wait_time_prioritized  = execute_Prioritized_plan(workspace_x, workspace_y, init_pos, goal_pos, tmp_workspace_2,  neighborDirectedGraph, workspace_1, robot_arrival_time, current_inflation_level)
    
    to_return = (planning_time_auction, plan_execution_time_auction, vcg_resolving_time, total_worst_scenarios, avg_path_auction, avg_wait_time_auction, planning_time_mstar_1, plan_execution_time_mstar_1, avg_path_mstar_1, avg_wait_time_mstar_1, planning_time_mstar_2, plan_execution_time_mstar_2, avg_path_mstar_2, avg_wait_time_mstar_2, planning_time_prioritized, max_path_prioritized, avg_path_prioritized, avg_wait_time_prioritized)
    
   
    return   to_return

    

if __name__ == '__main__':
    
    
        
    
    num_cores = multiprocessing.cpu_count()
    
    for obstacle_blocks in obstacle_block_number:
        workspace_size = []
    
        auction_plan_time = []
        auction_plan_execution_time = []
        auction_avg_plan_length = []
        auction_resolving_time = []
        auction_total_worst_scenarios = []
        auction_avg_wait_time = []
    
        mstar_plan_time_1 = []
        mstar_plan_execution_time_1 = []
        mstar_avg_plan_length_1 = []
        mstar_avg_wait_time_1 = []

        mstar_plan_time_2 = []
        mstar_plan_execution_time_2 = []
        mstar_avg_plan_length_2 = []
        mstar_avg_wait_time_2 = []
        
        prioritized_plan_time = []
        prioritized_plan_execution_time = []
        prioritized_avg_plan_length = []
        prioritized_avg_wait_time = []
        
        workspace_x = obstacle_blocks*(obstacle_block_size+4)+2
        workspace_y = obstacle_blocks*(obstacle_block_size+4)+2
        workspace_size.append(workspace_x)
        buildWorkspace(workspace_x, workspace_y,obstacle_block_size)
        valid_goal_positions, tmp_workspace_2, tmp_workspace_1, neighborDirectedGraph = generateWorkspace(workspace_x, workspace_y)
        
        for robots in robot_number:
            
            tmp_auction_plan_time = []
            tmp_auction_plan_execution_time = []
            tmp_auction_resolving_time = []
            tmp_auction_total_worst_scenarios = []
            tmp_auction_avg_plan_length = []
            tmp_auction_avg_wait_time = []
            
            tmp_mstar_plan_time_1 = []
            tmp_mstar_plan_execution_time_1 = []
            tmp_mstar_avg_plan_length_1 = []
            tmp_mstar_avg_wait_time_1 = []
            
            tmp_mstar_plan_time_2 = []
            tmp_mstar_plan_execution_time_2 = []
            tmp_mstar_avg_plan_length_2 = []
            tmp_mstar_avg_wait_time_2 = []
            
            tmp_prioritized_plan_time = []
            tmp_prioritized_plan_execution_time = []
            tmp_prioritized_avg_plan_length = []
            tmp_prioritized_avg_wait_time = []
            
            start_time = time.time() 
            results = Parallel(n_jobs=num_cores-2)(delayed(execute_path_planning)(workspace_x, workspace_y, robots, tmp_workspace_2, valid_goal_positions, neighborDirectedGraph, tmp_workspace_1) for i in range(0,number_of_iterations_per_robot_number))
            # To run the code serially
            #results = Parallel(n_jobs= 1)(delayed(execute_path_planning)(workspace_x, workspace_y, robots, tmp_workspace_2, valid_goal_positions, neighborDirectedGraph, tmp_workspace_1) for i in range(0,number_of_iterations_per_robot_number))
            end_time = time.time()
            
            print("Execution time:.......................... " + str(end_time - start_time))
            print("Result:")
            print(results)
            
            for tmp_result in results:
                tmp_auction_plan_time.append(tmp_result[0])
                tmp_auction_plan_execution_time.append(tmp_result[1])
                tmp_auction_resolving_time.append(tmp_result[2])
                tmp_auction_total_worst_scenarios.append(tmp_result[3])
                tmp_auction_avg_plan_length.append(tmp_result[4])
                tmp_auction_avg_wait_time.append(tmp_result[5])
                
              
                # M Star with inflation 1                
                tmp_mstar_plan_time_1.append(tmp_result[6])
                tmp_mstar_plan_execution_time_1.append(tmp_result[7])
                tmp_mstar_avg_plan_length_1.append(tmp_result[8])
                tmp_mstar_avg_wait_time_1.append(tmp_result[9])
                
                # M Star with inflation 2
                tmp_mstar_plan_time_2.append(tmp_result[10])
                tmp_mstar_plan_execution_time_2.append(tmp_result[11])
                tmp_mstar_avg_plan_length_2.append(tmp_result[12])
                tmp_mstar_avg_wait_time_2.append(tmp_result[13])
                
                # Prioritized Planning
                tmp_prioritized_plan_time.append(tmp_result[14])
                tmp_prioritized_plan_execution_time.append(tmp_result[15])
                tmp_prioritized_avg_plan_length.append(tmp_result[16])
                tmp_prioritized_avg_wait_time.append(tmp_result[17])
               
            auction_plan_time.append(tmp_auction_plan_time)
            auction_plan_execution_time.append(tmp_auction_plan_execution_time)
            auction_resolving_time.append(tmp_auction_resolving_time)
            auction_total_worst_scenarios.append(tmp_auction_total_worst_scenarios)
            auction_avg_plan_length.append(tmp_auction_avg_plan_length)
            auction_avg_wait_time.append(tmp_auction_avg_wait_time)
            
            
            # M Star with inflation 1
            mstar_plan_time_1.append(tmp_mstar_plan_time_1)
            mstar_plan_execution_time_1.append(tmp_mstar_plan_execution_time_1)
            mstar_avg_plan_length_1.append(tmp_mstar_avg_plan_length_1)
            mstar_avg_wait_time_1.append(tmp_mstar_avg_wait_time_1)
            
            # M Star with inflation 2
            mstar_plan_time_2.append(tmp_mstar_plan_time_2)
            mstar_plan_execution_time_2.append(tmp_mstar_plan_execution_time_2)
            mstar_avg_plan_length_2.append(tmp_mstar_avg_plan_length_2)
            mstar_avg_wait_time_2.append(tmp_mstar_avg_wait_time_2)
            
            # Prioritized Planning
            prioritized_plan_time.append(tmp_prioritized_plan_time)
            prioritized_plan_execution_time.append(tmp_prioritized_plan_execution_time)
            prioritized_avg_plan_length.append(tmp_prioritized_avg_plan_length)
            prioritized_avg_wait_time.append(tmp_prioritized_avg_wait_time)
            
            
        
           
        plotInfoFile = "data_for_plot_05_09_"+ str(workspace_x)+ ".txt"
        plotInfo = "data_for_plot_05_09_"+ str(workspace_x)
        f = open(plotInfoFile, 'w')
        f.write('Workspace_size = ' + repr([workspace_x]) + '\n')
        f.write('Robot_number = ' + repr(robot_number) + '\n')
        f.write('Auction_plan_time = ' + repr(auction_plan_time) + '\n')
        f.write('Auction_resolving_time = ' + repr(auction_resolving_time) + '\n') 
        f.write('Auction_total_worst_cases = ' + repr(auction_total_worst_scenarios) + '\n') 
        f.write('Auction_max_execution_time = ' + repr(auction_plan_execution_time) + '\n') 
        f.write('Auction_avg_execution_time = ' + repr(auction_avg_plan_length) + '\n')
        f.write('Auction_avg_wait_time = ' + repr(auction_avg_wait_time) + '\n')
        
        
        # M Star with inflation 1
        f.write('Mstar_plan_time_inflation_1 = ' + repr(mstar_plan_time_1) + '\n')
        f.write('Mstar_max_execution_time_inflation_1 = ' + repr(mstar_plan_execution_time_1) + '\n')
        f.write('Mstar_avg_execution_time_inflation_1 = ' + repr( mstar_avg_plan_length_1) + '\n')
        f.write('Mstar_avg_wait_time_inflation_1 = ' + repr(mstar_avg_wait_time_1) + '\n')
        
        # M Star with inflation 2
        f.write('Mstar_plan_time_inflation_2 = ' + repr(mstar_plan_time_2) + '\n')
        f.write('Mstar_max_execution_time_inflation_2 = ' + repr(mstar_plan_execution_time_2) + '\n')
        f.write('Mstar_avg_execution_time_inflation_2 = ' + repr( mstar_avg_plan_length_2) + '\n')
        f.write('Mstar_avg_wait_time_inflation_2 = ' + repr(mstar_avg_wait_time_2) + '\n')
        
        # Prioritized Planning
        f.write('Prioritized_plan_time = ' + repr(prioritized_plan_time) + '\n')
        f.write('Prioritized_max_execution_time = ' + repr( prioritized_plan_execution_time) + '\n')
        f.write('Prioritized_avg_execution_time = ' + repr( prioritized_avg_plan_length) + '\n')
        f.write('Prioritized_avg_wait_time = ' + repr(prioritized_avg_wait_time) + '\n')
        
        f.close()
        
        
        
       
        
        #### PLOTTING THE ERRORBAR FOR MEAN TIME
        ##### Mailing the data to an address

        f = open(plotInfoFile, 'r')
        
        for line in f.readlines():
            lineNew = line.replace('nan','np.nan')
            exec(lineNew)
        f.close()
        
        confidence = 0.95
        workspaceSize = Workspace_size[0]
        
        timePlanningAuctionMean = []
        timePlanningAuctionConf = []
        timePlanningAuctionSTD = []
        
        timeVCGReslovingMean = []
        timeVCGReslovingConf = []
        timeVCGReslovingSTD = []
    
        timeTotalAuctionMean = []
        timeTotalAuctionConf = []
        
        
        timePlanningMStarMean_1 = []
        timePlanningMStarConf_1 = []
        
        timePlanningMStarMean_2 = []
        timePlanningMStarConf_2 = []
        
        timePlanningPrioritizedMean = []
        timePlanningPrioritizedConf = []
        
        
        for index in range(len(Robot_number)):
            
            sampleMean = np.mean(Auction_plan_time[index])
            timePlanningAuctionMean.append(sampleMean)
            sampleConf = st.t.interval(confidence, len(Auction_plan_time[index])-1, loc=sampleMean, scale=st.sem(Auction_plan_time[index]))
            sampleError = (sampleMean - sampleConf[0], sampleConf[1] - sampleMean)
            timePlanningAuctionConf.append(sampleError)
            sampleSTD = np.std(Auction_plan_time[index])
            timePlanningAuctionSTD.append(sampleSTD)
            
            
            sampleMean = np.mean(Auction_resolving_time[index])
            timeVCGReslovingMean.append(sampleMean)
            sampleConf = st.t.interval(confidence, len(Auction_resolving_time[index])-1, loc=sampleMean, scale=st.sem(Auction_resolving_time[index]))
            sampleError = (sampleMean - sampleConf[0], sampleConf[1] - sampleMean)
            timeVCGReslovingConf.append(sampleError)
            sample_std = np.std(Auction_resolving_time[index])
            timeVCGReslovingSTD.append(sample_std)
            
            totalAuctionTime = list(map(lambda x, y: x + y, list(Auction_plan_time[index]), list(Auction_resolving_time[index])))
            sampleMean = np.mean(totalAuctionTime)
            timeTotalAuctionMean.append(sampleMean)
            sampleConf = st.t.interval(confidence, len(totalAuctionTime)-1, loc=sampleMean, scale=st.sem(totalAuctionTime))
            sampleError = (sampleMean - sampleConf[0], sampleConf[1] - sampleMean)
            timeTotalAuctionConf.append(sampleError)
                
  
            
            sampleMean = np.mean(Mstar_plan_time_inflation_1[index])
            timePlanningMStarMean_1.append(sampleMean)
            sampleConf = st.t.interval(confidence, len(Mstar_plan_time_inflation_1[index])-1, loc=sampleMean, scale=st.sem(Mstar_plan_time_inflation_1[index]))
            sampleError = (sampleMean - sampleConf[0], sampleConf[1] - sampleMean)
            timePlanningMStarConf_1.append(sampleError)
            
            # M Star with inflation 2
            sampleMean = np.mean(Mstar_plan_time_inflation_2[index])
            timePlanningMStarMean_2.append(sampleMean)
            sampleConf = st.t.interval(confidence, len(Mstar_plan_time_inflation_2[index])-1, loc=sampleMean, scale=st.sem(Mstar_plan_time_inflation_2[index]))
            sampleError = (sampleMean - sampleConf[0], sampleConf[1] - sampleMean)
            timePlanningMStarConf_2.append(sampleError)
            
            
            # Prioritized Planning
            sampleMean = np.mean(Prioritized_plan_time[index])
            timePlanningPrioritizedMean.append(sampleMean)
            sampleConf = st.t.interval(confidence, len(Prioritized_plan_time[index])-1, loc=sampleMean, scale=st.sem(Prioritized_plan_time[index]))
            sampleError = (sampleMean - sampleConf[0], sampleConf[1] - sampleMean)
            timePlanningPrioritizedConf.append(sampleError)
            
            
            
        titleText = 'Planning Time, Workspace Size = ' + str(workspaceSize)
        plt.figure('Comparison of SPARCAS and M*')
        plt.title(titleText)
        
        ax = plt.subplot(111)
        
        
        plt.errorbar(Robot_number, timeTotalAuctionMean, yerr=np.array(timeTotalAuctionConf).T, fmt='ko--', linewidth=2, capsize=2,markersize=8, label='SPARCAS')
               
        plt.errorbar(Robot_number, timePlanningMStarMean_1, yerr=np.array(timePlanningMStarConf_1).T, fmt='rd-', linewidth=2, capsize=2,markersize=8, label=r'$M^* Inflation = 1$')
        
        plt.errorbar(Robot_number, timePlanningMStarMean_2, yerr=np.array(timePlanningMStarConf_2).T, fmt='gx-.', linewidth=2, capsize=2,markersize=8, label=r'$M^* Inflation = 2$')
        
        plt.errorbar(Robot_number, timePlanningPrioritizedMean, yerr=np.array(timePlanningPrioritizedConf).T, fmt='b+:', linewidth=2, capsize=2,markersize=8, label=r'Prioritized Planning')
        
        ax.set_xlim(xmin=min(Robot_number)-1, xmax=max(Robot_number)+1)
        ax.set_ylabel('Planning Time (sec)', fontsize=15)
        ax.set_yscale("log", nonposy='clip')
        
        ax.set_xlabel(r'Number of Robots', fontsize=15)
       
        ax.grid()
        
        plt.legend(loc='best')
        
        figure = plt.gcf() # get current figure
        figure.set_size_inches(10, 5)
      
        
        plt.savefig(plotInfo+'.png', dpi=100)
        plt.close('all')
        
        ind = np.arange(len(Robot_number))  # the x locations for the groups
        width = 0.20    # the width of the bars
        gap = 0.05
        fig, ax = plt.subplots()
        
        
        
       
        plan_mean = ax.bar(ind + gap, timePlanningAuctionMean, width, color='g',yerr=timePlanningAuctionSTD, align='center', alpha=0.5, ecolor='black', capsize=10,log=True)
        vcg_mean = ax.bar(ind + gap, timeVCGReslovingMean, width, color='b',bottom=timePlanningAuctionMean, yerr=timeVCGReslovingSTD, align='center', alpha=0.5, ecolor='black', capsize=10,log=True)
              
       
        ax.set_ylim(ymin=0.0001)
       
        ax.set_ylabel('Time (Seconds)',fontsize=15, color='k')
        ax.set_xlabel(r'Number of Roads',fontsize=15, color='k')
       
        
        plt.xticks(ind+gap, Robot_number)
        plt.rcParams["figure.figsize"] = (9,5)
        
     
        ax.legend( (plan_mean[0], vcg_mean[0]), ('Time for Path Planning', 'Time for Resolving VCG'),fontsize=10,loc='upper left',fancybox=True, framealpha=0.8)
        totalAuctionTime = list(map(lambda x, y: x + y, list(timePlanningAuctionMean), list(timeVCGReslovingMean)))
        max_total_time = max(totalAuctionTime) 
        
        plt.ylim(0,max_total_time*10.0)
        
        
        plt.grid(True)
       
        plt.savefig(plotInfo+'PlanningTimeBar.png', dpi=100)
        plt.close('all')
        
           
        
        """
        ### Code to email the result  ...
        fromaddr = "my.python.notifier@gmail.com"          
                
        toaddr = ["destination_email_address"]
        msg = MIMEMultipart()
        
        msg['From'] = fromaddr
        msg['To'] = ", ".join(toaddr)
        msg['Subject'] = "Your simulation on "+plotInfo+" is complete"
        
        body = "The files are attached. Enjoy your day/night!"
        
        msg.attach(MIMEText(body, 'plain'))
        
        filename1 = plotInfo+'.txt'
        attachment1 = open(filename1, "rb")
        
        part1 = MIMEBase('application', 'octet-stream')
        part1.set_payload((attachment1).read())
        encoders.encode_base64(part1)
        part1.add_header('Content-Disposition', "attachment; filename= %s" % filename1)
        
        msg.attach(part1)
        attachment1.close()
        
        
        filename2 = plotInfo+'.png'
        attachment2 = open(filename2, "rb")
        
        part2 = MIMEBase('application', 'octet-stream')
        part2.set_payload((attachment2).read())
        encoders.encode_base64(part2)
        part2.add_header('Content-Disposition', "attachment; filename= %s" % filename2)
        
        msg.attach(part2)
        attachment2.close()
        
        
        filename3 = plotInfo+'PlanningTimeBar.png'
        attachment3 = open(filename3, "rb")
        
        part3 = MIMEBase('application', 'octet-stream')
        part3.set_payload((attachment3).read())
        encoders.encode_base64(part3)
        part3.add_header('Content-Disposition', "attachment; filename= %s" % filename3)
        
        msg.attach(part3)
        attachment3.close()
        
             
           
   
        server = smtplib.SMTP('smtp.gmail.com', 587)
        server.ehlo()
        server.starttls()
        server.login("my.python.notifier@gmail.com", "nohtyp$123")
        text = msg.as_string()
        server.sendmail(fromaddr, toaddr, text)
        server.quit()
        # EMAIL REMINDER OF THE TERMINATION
        """
        
       
        
        
            
            
    
    
    
    
        
    
