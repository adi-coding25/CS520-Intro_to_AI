# -*- coding: utf-8 -*-
"""
Created on Sat Oct 16 13:12:07 2021

@author: sachi
"""

#Agent 1

'''
Variables:
dim - dimension
prob - probablity
visited {} - visited nodes

'''

import numpy as np
import random
import heapq
from math import sqrt,pow
import time

def create_gridworld(dim,prob):
    shape = (dim,dim)
    arr = (np.zeros(shape))

    for i in range(dim):
        for j in range(dim):
            x = random.random()
            if (x<prob):
                arr[i][j]=-1
       
    arr[0][0]=0
    arr[dim-1][dim-1]=0
    return arr
    
 


def compute_heuristic_manhattan(l):
    arr = (np.zeros(l.shape))
    for i in range(l.shape[0]):
        for j in range(l.shape[1]):
            arr[i][j] = (abs((l.shape[0]-1)-i)+abs((l.shape[1]-1)-j))
    return arr

#h=compute_heuristic_manhattan(l)
#print(h)

#parent = {}
#b = (np.zeros(l.shape))
#b = l

# Arguments - x,y -- Curent position | b - knowledge base of the grid as of now | visited nodes | parent - parent node (globally updated)
def get_children(x,y,b,visited,parent):
    children = []
    
    # If within limits and not blocked and not visited already -- add as child and update x,y as parent of it
    if(x+1<b.shape[0]) and b[x+1, y]!= -1 and visited[(x+1,y)] == False:
        children.append((x+1,y))
        parent[(x+1,y)] = (x,y)

    if(y+1<b.shape[1]) and b[x,y+1] != -1 and visited[(x,y+1)] == False:
        children.append((x,y+1))
        parent[(x,y+1)] = (x,y)
        
    if(x-1>=0) and b[x-1, y] != -1 and visited[(x-1,y)] == False:
        children.append((x-1,y))
        parent[(x-1,y)] = (x,y)
        
    if(y-1>=0) and b[x, y-1] != -1 and visited[(x,y-1)] == False:
        children.append((x,y-1))
        parent[(x,y-1)] = (x,y)
        
    return children


def increment_g(g,x,y,i,j):
    g[(x,y)] = g[(i,j)]+1
    #print(g)


# c - children list, q - priority queue implemented via heaps, h - heuristic, g - cost till visit, pi.pj - parents of children in c
def pqadd(c,q,h,g,pi,pj):
  #for each children
    for i in c:
      #increment cost of all children +1
        increment_g(g,i[0],i[1],pi,pj)
        #add childs to q with (total planned cost, heuristic, child (x,y))
        heapq.heappush(q, (h[i]+g[i],h[i], i))
    
  
#q = []


# l - actual gridworld (with info), h - heuristic to goal of grid,  i.j - position of start, b - knowledge base
def astarplan(l,h,i,j,m,n,b):

    visited = {}
    for m in range(0,101):
      for n in range(0,101):
        visited[(m,n)] = False  #visited nodes
    

    parent = {}
    g = (np.zeros(l.shape)) #grid shaped cost to node - initially set to zero and updated when visited
    q = []
    #global b

    si = i #start point of x coordinate - current plan
    sj = j #start point of y coordinate - current plan

    while(i<l.shape[0] and j<l.shape[1]):   #fix this
        
        
        c = get_children(i,j,b,visited,parent)
        
        pqadd(c,q,h,g,i,j)
        
    
        if len(q) == 0:
          #if no path exists - return -1 
          return -1,b
        #pop the top child
        qpop = heapq.heappop(q)
        
        #position of child
        i,j = qpop[2]
        
        #visited i,j is true
        visited[(i,j)] = True

        
        if((i,j) == (l.shape[0]-1,l.shape[1]-1)): #Goal
            f = (i,j)
            #set path to empty
            path = []
            #add final node to path
            path.append((l.shape[0]-1,l.shape[1]-1))
            
            #While not reaching to start of a star, backtrack and add to path
            while(parent[f]!=(si,sj)):
              f = (parent[f])
              path.append(f)

            #Add initial to path
            path.append((si,sj))
            #reverse the list to get the path
            path.reverse()
    
            #return path and the updated knowledge base
            return path,b      


        
    q = []
    g = (np.zeros(l.shape))

# l - actual gridworld, h - heuristic of all nodes, path - planned path, b - knowledge base
def astarexecute(l,h,path,b):

  # If goal is unreachable return -1
  if path == -1:
    return path
  #global b

  fake_visited = {}
  fake_parent = {}

  for m in range(0,101):
      for n in range(0,101):
        fake_visited[(m,n)] = False  #set fake visited all false
  
  # for coordinates of position in path
  for i in range(len(path)):

    # if actual path element is blocked we return the last unblocked element
    if(l[path[i]] == -1):
      return path[i-1]
    
    #while executing, update the knowledge base based on the neighbors
    c = get_children(path[i][0],path[i][1],b,fake_visited,fake_parent)
    for j in c:
      #if neighbor blocked in actual path - update knowledge base about it
      if l[j] == -1.0:
        b[j] = -1.0

  # return position of final traversed path - i.e goal
  return (path[i])

def exampleAgent(l,h,path,b):
    
    return

def repeatedastar(start,l,h,b):

  if start == (l.shape[0]-1,l.shape[1]-1):
    return True

  if start == -1:
    return False


  path,b = astarplan(l,h,start[0],start[1],b)
  x = astarexecute(l,h,path,b)
  
  return repeatedastar(x,l,h,b)



dim = 101
prob = 0.25
l = create_gridworld(dim,prob)
h=compute_heuristic_manhattan(l)
x = (np.zeros(shape=(7,dim,dim)))
b=x[:][:][2]
parent = {}
q = []
start_time = time.time()
print(repeatedastar((0,0),l,h,b))
print("Time taken : ", (time.time() - start_time))




