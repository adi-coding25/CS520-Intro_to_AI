#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Oct 15 20:24:00 2021

@author: aditya
"""

#Agent 4
import numpy as np
import heapq
import random
from math import sqrt,pow
import asp
import time
'''
This program contains the code for our agent i.e. Agent 4. 
Variables:
    grid -> Contains the actual gridworld
    start-> Contains the coordinates of starting location/node
    end->   Contains the coordinates of ending location/node (or goal node)
    n->     Dimension of the grid
    h->     Heuristics of each node to the goal for the whole grid
    cellnworld->It is our knowlledge base and contains following info:
                    -the number of neighbors current node has
                    -Whether or not current node has been visited
                    -Whether or not current node has been confirmed as empty or blocked, or is currently unconfirmed
                    -the number of neighbors of current node that are sensed to be blocked
                    -the number of neighbors of current node that have been confirmed to be blocked
                    -the number of neighbors of current node that have been confirmed to be empty 
                    -the number of neighbors of current node that are still hidden or unconfirmed either way.
    path->  Contains the planned path
    count1->Contains the count of number of nodes visited
    count2->Contains the count of number of nodes in final trajectory
'''
class ourAgent:
    
    def __init__(self,grid,start,end,h,n):
        self.grid=grid
        self.start=start
        self.end=end
        self.n=n
        self.h=h
        self.cellnworld = np.zeros(shape=(7,n,n))
        self.path=[]
        self.count1=self.count2=0
    
    '''
    checkNeighborCount()-> Counts the number of neighbors of given nodee
    
    Input: pos-> Coordinates of current node
    
    Returns the count of the number of neighbors
    '''
    def checkNeighborCount(self,pos):
        count=8
        if pos[0]==0 or pos[0]==self.n-1:
            if pos[1]==0 or pos[1]==self.n-1:
                count=3
            else:
                count=5
        return count
    
    '''
    partialSensing()-> Senses the number of blocks in the neighbor of current node
    
    Input: pos-> Coordinates of current node
    
    Returns the count of the number of blocks in the neighboring area
    '''
    def partialSensing(self,pos):
        count=0
        for i in range(-1,2):
            for j in range(-1,2):
                if self.isValid([pos[0]+i,pos[1]+j]):
                    if self.grid[pos[0]+i][pos[1]+j]:
                        count+=1
        return count
    
    '''
    isBlocked()-> Checks whether the node bring visited is blocked or not
    
    Input: pos-> Coordinates of current node
    
    Updates the knowledge base and returns boolean based on blocked status
    '''
    def isBlocked(self,pos):
        if self.grid[pos[0]][pos[1]]==-1:
            self.cellnworld[2][pos[0]][pos[1]]=-1
            return 1
        else:
            self.cellnworld[2][pos[0]][pos[1]]=1
            return 0
    
    '''
    checkNeighbor()-> It initialises the current node by following operations:
                        - initializes the count of number of neighbors
                        - initializes the count of number of neighbors not explored
                        - initializes the count of number of neighbors sensed to be blocked
                        - sets node as visited
                        - counts the number of neighbors already confirmed to be blocked
                        - counts the number of neighbors already confirmed to be empty
    
    Input: pos-> Coordinates of current node
    '''
    def checkNeighbor(self,pos):
        self.cellnworld[0][pos[0]][pos[1]]=self.cellnworld[6][pos[0]][pos[1]]=self.checkNeighborCount(pos)
        self.cellnworld[3][pos[0]][pos[1]]=self.partialSensing(pos)
        self.cellnworld[1][pos[0]][pos[1]]=1
        countb=counte=0
        for i in range(-1,2):
            if pos[0]+i>=0 and pos[0]+i<self.n:
                for j in range(-1,2):
                    if pos[1]+j>=0 and pos[1]+j<self.n:
                        if self.cellnworld[2][pos[0]+i][pos[1]+j]==-1:
                            countb+=1
                        elif self.cellnworld[2][pos[0]+i][pos[1]+j]==1:
                            counte+=1
        self.cellnworld[4][pos[0]][pos[1]]=countb
        self.cellnworld[5][pos[0]][pos[1]]=counte
        self.cellnworld[6][pos[0]][pos[1]]-=(countb+counte)
        
    '''
    isDest()-> checks whether the start node is the goal node
    '''     
    def isDest(self):
        if self.start==self.end:
            return True
        else:
            return False
    
    '''
    mainAgent()-> The first function called to run agent 4. It performs following set of operations:
                    - checks isDest(), if True exits
                    - initialises the start node as current node
                    - computes a path from current node to goal
                    - if no path exists, exits
                    - calls exploreAgent() to explore the path found
                    - loops through steps 3 through 5 till final destination is found or no path exists.
    
    Returns whether there is a true path to goal from start.
    '''
    def mainAgent(self):
        if self.isDest():
            return -1
        p=self.start
        x=False
        while x==False: 
            self.checkNeighbor(p)
            self.path,c=asp.astarplan(self.grid, self.h, p[0], p[1], self.cellnworld[:][:][2])
            #print(self.path)
            if self.path==-1:
                return False
            x,p=self.exploreAgent(p)
        return x
    
    '''
    movingBlock()-> tries to determine whether the neighboring node to be visited is blocked or not as follows:
                        - checks whether the node has already been visited, if yes, returns True and does no further computation
                        - checks the ratio of no. of neighboring blocks yet to be explored / no. of neighbors yet to be explored
                        - if this ratio is below 0.67, dont mark the node to be visited as blocked and return true
                        - else mark the node as blocked
        
    Input:      p-> current node
                q-> neighboring node for which computation is bring done
    '''
    def movingBlock(self,p,q):
        if self.cellnworld[1][q[0]][q[1]]:
            return True
        bp=(self.cellnworld[3][p[0]][p[1]]-self.cellnworld[4][p[0]][p[1]])/self.cellnworld[6][p[0]][p[1]]
        if bp<0.67:
            return True
        else:
            return False
    
    '''
    exploreAgent()-> This function explores the computed path as follows:
                        - pop the first node from path and store in q
                        - check whether q is the goal node, if yes return True and current node
                        - call movingBlocks() to compute whether q should be determined as blocked or not
                        - if movingBlocks() is True, check whether q is blocked in actual gridworld, if yes, update knowledge base variables and return False and parent of q i.e. p
                        - If not blocked, then update knowldge base accordingly, call checkNeighbor() on q and set parent p as q
                        - repeat steps 1 through 5 till the computed path is empty or goal is found
    
    Input:  p-> current node
    '''
    def exploreAgent(self,p):
        while self.path!=[]:
            q=self.path.pop(0)
            self.count1+=1
            if q==tuple(self.end):
                return True,q
            i,j=q[0],q[1]
            #enter code here
            if self.movingBlock(p, q):
                if self.isBlocked(q)==1:
                    self.cellnworld[4][p[0]][p[1]]+=1
                    self.cellnworld[6][p[0]][p[1]]-=1
                    self.cellnworld[1][q[0]][q[1]]=1
                    return False,p
                self.cellnworld[5][p[0]][p[1]]+=1
                self.cellnworld[6][p[0]][p[1]]-=1
                self.checkNeighbor(q)
                p=q
            else:
                self.cellnworld[2][q[0]][q[1]]=-1
                self.cellnworld[4][p[0]][p[1]]+=1
                self.cellnworld[6][p[0]][p[1]]-=1
                self.cellnworld[1][q[0]][q[1]]=1
                return False,p
    '''
    isValid()-> checks whether the current node is valid (i.e. within the dimensions)
    
    Input: pos-> Coordinates of current node
    
    Returns boolean depending on the validity
    '''      
    def isValid(self,pos):
        if pos[0]>=0 and pos[0]<self.n and pos[1]>=0 and pos[1]<self.n:
            return True
        else:
            return False
    
    '''
    finalRun()-> computes the final trajectory length
    '''
    def finalRun(self):
        p=self.start
        self.path,c=asp.astarplan(self.grid, self.h, p[0], p[1], self.cellnworld[:][:][2])
        if self.path!=-1:
            self.count2+=len(self.path)
        else:
            self.count2=-1
    
'''
create_gridlock()-> creates the gridworld through which the path is to be computed

Inputs:     dim-> dimension of the gridworld
            p->   probablity of block

Returns the gridworld
'''
def create_gridlock(dim,p):
    shape = (dim,dim)
    arr = (np.zeros(shape))
    
    for i in range(dim):
        for j in range(dim):
            x = random.random()
            if (x<p):
                arr[i][j]=-1
       
    arr[0][0]=0
    arr[dim-1][dim-1]=0
    return arr

'''
manhattan_dist()-> computes the manhattan distance between the current node and the goal

Input:      grid-> actual gridworld
            x,y-> coordinates of current node

Returns the manhattan distance
'''
def manhattan_dist(grid,x,y):
      return (abs((grid.shape[0]-1)-x)+abs((grid.shape[1]-1)-y))

'''
compute_heuristic_manhattan()-> computes the heuristics (using manhattan distance) for each node in the whole gridworld

Input:      grid-> actual gridworld

Returns 2D array containing heuristics of complete gridworld
'''
def compute_heuristic_manhattan(grid):
    arr = (np.zeros(grid.shape))
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            arr[i][j] = manhattan_dist(grid,i,j)
    return arr

start_time = time.time()
for i in range(1):
    grid=create_gridlock(101,0.33)
    start=[0,0]
    end=[100,100]
    n=101
    h=compute_heuristic_manhattan(grid)
    ea=ourAgent(grid, start, end, h, n)
    ea.mainAgent()
    print(ea.count1)
    ea.finalRun()
    print(ea.count2)
print("Time taken : ", (time.time() - start_time))

