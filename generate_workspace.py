import sys
import time as timer
import random
import time
import numpy as np
import networkx as nx
import pandas as pd
import matplotlib.pyplot as plt
from colors import color
from PIL import Image


def buildWorkspace(workspace_x,workspace_y,obstacle_block_size):
    arr=np.empty((workspace_x,workspace_y), dtype='object')
    

    for j in range(workspace_y-1,-1,-(obstacle_block_size+4)):
        for i in range(0,workspace_x,1):
            if((j%2) == 1):
                arr[i][j] = 'L'
                arr[i][j-1] = 'R'
            else:
                arr[i][j] = 'L'
                arr[i][j-1] = 'R'

    for j in range(workspace_y-1,-1,-1):
        for i in range(0,workspace_x-1,(obstacle_block_size+4)):
            if((i%2) == 0):
                arr[i][j] = 'D'
                arr[i+1][j] = 'U'
            else:
                arr[i][j] = 'D'
                arr[i+1][j] = 'U'

    for i in range(0,workspace_x-1,obstacle_block_size+4):
        for j in range(0,workspace_y-1,obstacle_block_size+4):
            arr[i][j] = 'RD'
            arr[i+1][j] = 'RU'
            arr[i][j+1] = 'LD'
            arr[i+1][j+1] = 'LU'

    for i in range(0,workspace_x-1,obstacle_block_size+4):
        arr[i][0] ='RR'

    for i in range(0,workspace_x-1,obstacle_block_size+4):
        arr[i+1][workspace_y-1] ='LL'


    for j in range(0,workspace_y-1,obstacle_block_size+4):
        arr[0][j+1] ='DD'

    for j in range(0,workspace_y-1,obstacle_block_size+4):
        arr[workspace_x-1][j] ='UU'


    arr[0][0] = 'RR'
    arr[workspace_x-1][0] = 'UU'
    arr[0][workspace_y-1] = 'DD'
    arr[workspace_x-1][workspace_y-1] = 'LL'



    for i in range(2,workspace_x-2,(obstacle_block_size+4)):
        for j in range(2,workspace_y-2,(obstacle_block_size+4)):
            for k in range(0,2+obstacle_block_size,1):
                for l in range(0,2+obstacle_block_size,1):
                    arr[i+k][j+l] = 'S'

    for i in range(3,workspace_x-3,(obstacle_block_size+4)):
        for j in range(3,workspace_y-3,(obstacle_block_size+4)):
            for k in range(0,obstacle_block_size,1):
                for l in range(0,obstacle_block_size,1):
                    arr[i+k][j+l] = 'O'
    
 


    for j in range(workspace_y-1,-1,-1):
        for i in range(0,workspace_x,1):
            print(str(arr[i][j])+" ", end = "")
        print("")


    """print(".........................")
    for j in range(workspace_y-1,-1,-1):
        for i in range(0,workspace_x,1):
            print("( "+ str(i)+","+str(j)+")",  end = "")
        print("")"""

    path='route.txt'
    file=open(path,'w')
    for i in range(workspace_x):
        for j in range(workspace_y):
            s=arr[i][j]
            file.write(str(i)+','+str(j)+':'+str(s)+'\n')
            #print("("+str(i)+","+str(j)+")",end="")
            #print("\n")
            #print(arr[i])
    file.close()
    
    
def getWorkspace(workspace_x, workspace_y):
    #global matrix
    matrix=[[0]*workspace_y for i in range(workspace_x)]
    valid_init_pos = []
    valid_goal_pos = []
    f=open('route.txt','r')
    while(1):
        flag=0
        val=80
        s=f.readline()
        if ""==s:
            break
        if s[3]==',':
            if s[7]==':':
                x=8
                i=100*int(s[0]) + 10*int(s[1]) + int(s[2])
                j=100*int(s[4]) + 10*int(s[5]) + int(s[6])
            elif s[6]==':':
                x=7
                i=100*int(s[0]) + 10*int(s[1]) + int(s[2])
                j=10*int(s[4])+int(s[5])
            elif s[5]==':':
                x=6
                i=100*int(s[0]) + 10*int(s[1]) + int(s[2])
                j=int(s[4])
        elif s[2]==',':
            if s[6]==':':
                x=7
                i=10*int(s[0])+int(s[1])
                j=100*int(s[3]) + 10*int(s[4]) + int(s[5])
            elif s[5]==':':
                x=6
                i=10*int(s[0])+int(s[1])
                j=10*int(s[3])+int(s[4])
            elif s[4]==':':
                x=5
                i=10*int(s[0])+int(s[1])
                j=int(s[3])
        elif s[1]==',':
            if s[5]==':':
                x=6
                i=int(s[0])
                j=100*int(s[2]) + 10*int(s[3]) + int(s[4])
            elif s[4]==':':
                x=5
                i=int(s[0])
                j=10*int(s[2])+int(s[3])
            elif s[3]==':':
                x=4
                i=int(s[0])
                j=int(s[2])
         
            
        if s[x]=='L'and s[x+1]=='\n':
            val=1
        if s[x]=='R'and s[x+1]=='\n':
            val=2
        if s[x]=='U'and s[x+1]=='\n':
            val=3
        if s[x]=='D'and s[x+1]=='\n':
            val=4
        if s[x]=='O'and s[x+1]=='\n':
            flag=-1
            val=-1
        if s[x]=='S'and s[x+1]=='\n':
            val=0
            
        if s[x]=='L' and s[x+1]=='U':
            val=5
        if s[x]=='L'and s[x+1]=='D':
            val=6
        if s[x]=='R' and s[x+1]=='U':
            val=7
        if s[x]=='R' and s[x+1]=='D':
            val=8
        if s[x]=='L' and s[x+1]=='L':
            val=9
        if s[x]=='R' and s[x+1]=='R':
            val=10
        if s[x]=='U' and s[x+1]=='U':
            val=11
        if s[x]=='D' and s[x+1]=='D':
            val=12
        matrix[i][j]=val
        if flag==0:
            x=[i,j]
            if(val>0):               
                valid_init_pos.append(x)
            else:
                valid_goal_pos.append(x)
    f.close()
    return matrix, valid_init_pos, valid_goal_pos


def getNeighborGraph(workspace_x, workspace_y, matrix):
    nG=nx.DiGraph()
    nG.add_nodes_from([1,workspace_x*workspace_y])
    for b in range(1,workspace_x*workspace_y+1):
        x=int((b-1)%workspace_x)
        y=int((b-1)/workspace_y)
        val=matrix[x][y]
        if(val < 0):
          continue
        if(val > 0):
            nG.add_edge(b,b,weight=1)   
        if((val == 1) or (val == 9)):
           nG.add_edge(b,b-1,weight=1)
        if((val == 2) or (val == 10)):
           nG.add_edge(b,b+1,weight=1)
        if((val == 3) or (val == 11)):
           nG.add_edge(b,b+workspace_x,weight=1)
        if((val == 4) or (val == 12)):
           nG.add_edge(b,b-workspace_x,weight=1)

        if(val == 5):
           nG.add_edge(b,b-1,weight=1)
           nG.add_edge(b,b+workspace_x,weight=1)
        if(val == 6):
           nG.add_edge(b,b-1,weight=1)
           nG.add_edge(b,b-workspace_x,weight=1)
        if(val == 7):
           nG.add_edge(b,b+1,weight=1)
           nG.add_edge(b,b+workspace_x,weight=1)
        if(val == 8):
           nG.add_edge(b,b+1,weight=1)
           nG.add_edge(b,b-workspace_x,weight=1)

        if(val == 0):
          nG.add_edge(b,b,weight=1)  
          tmp_x = x + 1
          tmp_y = y
          flag = 1
          if((matrix[tmp_x][tmp_y] >= 1) and flag == 1):
              nG.add_edge(b+1,b,weight=1.5)
              nG.add_edge(b,b+1, weight=1.5)
              flag = 0 
          tmp_x = x - 1
          tmp_y = y
          if((matrix[tmp_x][tmp_y] >= 1) and flag == 1):
              nG.add_edge(b-1,b,weight=1.5)
              nG.add_edge(b,b-1,weight=1.5)
              flag = 0 
          tmp_x = x 
          tmp_y = y - 1
          if((matrix[tmp_x][tmp_y] >= 1) and flag == 1):
              nG.add_edge(b-workspace_x,b,weight=1.5)
              nG.add_edge(b,b-workspace_x,weight=1.5)
              flag = 0  
          tmp_x = x 
          tmp_y = y + 1
          if((matrix[tmp_x][tmp_y] >= 1) and flag == 1):
              nG.add_edge(b+workspace_x,b,weight=1.5)
              nG.add_edge(b,b+workspace_x,weight=1.5)
              flag = 0
              
    
    return nG    
    
def buildWorkspaceImage(workspace_x,workspace_y,workspace_1):

    tmp_workspace_list = []
    workspace_matrix=np.zeros((workspace_x, workspace_y, 3), dtype=np.uint8)
    print("..................Tmp_workspace_1.................") 
    for j in range(workspace_y-1,-1,-1):
         for i in range(0,workspace_x,1):
              tmp_workspace_list.append(workspace_1[i][j])
              print(str(workspace_1[i][j])+" ", end = "")
         print("")
              
    for i in range(0,workspace_x,1):
        for j in range(0,workspace_y,1):
            tmp_index = i*workspace_x + j
            if(tmp_workspace_list[tmp_index] == -1):
                workspace_matrix[i][j] = [0,0,0]
            else:  
                workspace_matrix[i][j] = [255,255,255]  
                     
         
         
    #print(workspace_matrix)     
    img = Image.fromarray(workspace_matrix, 'RGB')
    img.save('my.png')
    img.show()
