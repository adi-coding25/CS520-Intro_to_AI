#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov  9 15:51:01 2021

@author: aditya
"""
import gridworld as gw
import numpy as np
import math
import random
import asp

'''
This program contains the code for Agent 8. 
Variables:
    grid -> Contains the actual gridworld
    start-> Contains the coordinates of starting location/node
    n->     Dimension of the grid
    h->     Heuristics of each node to the goal for the whole grid
    ex,ey-> Contains the coordinates of randomly selected ending location/node (or goal node) 
    arb_end->Contains the coordinates of arbitrarily selected end node, which hass een found by max of probablity
    kb->    It is our knowledge base and contains info about probablity
    path->  Contains the planned path
    mc->    Count of the number of movements
    ec->    Count of the number of examinations
    ukb->   Probablity of containing the goal based on terrain
    mukb->  Probablity of containing the goal based on terrain * Probablity in knowledge base
    s->     Contains the distance of each cell from current start node
'''
class ourAgent:
    
    def __init__(self, n, grid, ex, ey, kb):
        self.n=n
        self.start=[n//2,n//2]
        shape = (n,n)
        self.grid=grid
        self.h=np.zeros(shape)
        self.s=np.zeros(shape)
        self.ex,self.ey=ex,ey
        self.arb_end=[0,0]
        self.kb=kb
        self.ukb=np.zeros(shape)+0.5
        self.mukb=np.zeros(shape)
        self.mc=0
        self.ec=0

    def distStart(self,p):
        for i in range(self.n):
            for j in range(self.n):
                if i==p[0] and j==p[1]:
                    self.s[i][j]=self.n+1
                else:
                    self.s[i][j]=abs(p[0]-i)+abs(p[1]-j)

    def manhattan_dist(self,x,y):
        return abs(self.arb_end[0]-x)+abs(self.arb_end[1]-y)
      
    def findMax(self):
        for i in range(self.n):
            for j in range(self.n):
                self.mukb[i][j]=(self.kb[i][j]*self.ukb[i][j])/math.pow(self.s[i][j],1.5)
        mi=np.where(self.mukb==np.amax(self.mukb))
        self.max_index=list(zip(mi[0], mi[1]))
    
    def compute_heuristic_manhattan(self):
        arr = (np.zeros(self.grid.shape))
        for i in range(self.grid.shape[0]):
            for j in range(self.grid.shape[1]):
                arr[i][j] = self.manhattan_dist(i,j)
        return arr
    
    def isDest(self):
        if self.start==self.arb_end:
            return True
        else:
            return False
    
    def isBlocked(self,pos):
        if self.grid[pos[0]][pos[1]]==-1:
            temp=self.kb[pos[0]][pos[1]]
            self.kb[pos[0]][pos[1]]=0
            self.ukb[pos[0]][pos[1]]=0
            return True,temp
        else:
            return False,0
    
    def terrainFlat(self,pos):
        temp=self.kb[pos[0]][pos[1]]
        self.kb[pos[0]][pos[1]]=(0.2*temp)/(1-0.8*temp)
        self.ukb[pos[0]][pos[1]]=0.8
        for i in range(self.kb.shape[0]):
            for j in range(self.kb.shape[1]):
                self.kb[i][j]+=(self.kb[i][j]/(1-temp))*(temp-self.kb[pos[0]][pos[1]])
        pass
    
    def terrainHill(self,pos):
        temp=self.kb[pos[0]][pos[1]]
        self.kb[pos[0]][pos[1]]=(0.5*temp)/(1-0.5*temp)
        self.ukb[pos[0]][pos[1]]=0.5
        for i in range(self.kb.shape[0]):
            for j in range(self.kb.shape[1]):
                self.kb[i][j]+=(self.kb[i][j]/(1-temp))*(temp-self.kb[pos[0]][pos[1]])
        pass
    
    def terrainForest(self,pos):
        temp=self.kb[pos[0]][pos[1]]
        self.kb[pos[0]][pos[1]]=(0.8*temp)/(1-0.2*temp)
        self.ukb[pos[0]][pos[1]]=0.2
        for i in range(self.kb.shape[0]):
            for j in range(self.kb.shape[1]):
                self.kb[i][j]+=(self.kb[i][j]/(1-temp))*(temp-self.kb[pos[0]][pos[1]])
        pass
      
    def checkCell(self,pos,ct):
        if self.ex==pos[0] and self.ey==pos[1]:
            return True
        else:
            self.updateBlockProb(ct)
            if self.grid[pos[0]][pos[1]]==1:
                self.terrainFlat(pos)
            elif self.grid[pos[0]][pos[1]]==2:
                self.terrainHill(pos)
            elif self.grid[pos[0]][pos[1]]==3:
                self.terrainForest(pos)
            return False
        
        
    def explorer(self):
        x=False
        p=self.start
        count=0
        while x==False:
            self.path,c=asp.astarplan(self.grid, self.h, p[0], p[1], self.arb_end[0], self.arb_end[1], self.kb)
            if self.path==-1:
                return False
            while self.path!=[]:
                k=False
                q=self.path.pop(0)
                if q==self.arb_end:
                    k=True
                m,t=self.isBlocked(q)
                self.mc+=1
                if m==True and k==True:
                    count+=t
                    self.findArbEnd()
                    print(self.kb[q[0]][q[1]],self.kb[p[0]][p[1]])
                    #write code for path planning from p
                    break
                elif m==True:
                    count+=t
                    break
                p=q  
            if m==True:
                continue
            x=self.checkCell(p,count)
            count=0
            self.distStart(p)
            self.findArbEnd()
            self.ec+=1
        return x,self.ec,self.mc
            
            
    def updateBlockProb(self,ct):
        for i in range(self.kb.shape[0]):
            for j in range(self.kb.shape[1]):
                self.kb[i][j]/=(1-ct)
                    
    def findArbEnd(self):
        self.findMax()
        if len(self.max_index)>1:
            x=[]
            for i in self.max_index:
                x.append(self.manhattan_dist(i[0], i[1]))
            m=min(x)
            c=x.count(m)
            if c==1:
                self.arb_end=self.max_index[x.index(m)]
            else:
                y=[]
                for i in range(len(x)):
                    if m==x[i]:
                        y.append(self.max_index[i])
                self.arb_end=random.choice(y)
        else:
           self.arb_end=self.max_index[0]
        self.h=self.compute_heuristic_manhattan()
    
    def agent(self):
        self.distStart(self.start)
        self.findArbEnd()
        print(self.arb_end)
        self.path,c=asp.astarplan(self.grid, self.h, self.start[0], self.start[1], self.arb_end[0], self.arb_end[1], self.kb)
        if self.path==-1:
            return False
        else:
            return True


def createGrid(n):
    grid,ex,ey=gw.create_gridworld(n)
    kb=np.zeros((n,n))
    for i in range(n):
        for j in range(n):
            kb[i][j]=1/math.pow(n,2)
    return grid,ex,ey,kb

n=101
grid,ex,ey,kb=createGrid(n)
oa=ourAgent(n,grid,ex,ey,kb)
x=oa.agent()
print(x)
if x:
    b,e,m=oa.explorer()
    if b:
        print("Path found",e,m)
    