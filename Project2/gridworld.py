# -*- coding: utf-8 -*-
"""
Created on Tue Nov  9 14:11:17 2021

@author: sachi
"""
import numpy as np
import random


# Creates an 2-D array of dim*dim with values -1 -> blocked (prob 0.3)
# Unblocked with different terrains (equal prob 0.7/3) 
# 1-> Flat   2-> Hilly   3-> Thick Forest
def create_gridworld(dim):
    shape = (dim,dim)
    arr = (np.zeros(shape))
    
    #arr = [[0]*dim]*dim

    for i in range(dim):
        for j in range(dim):
            x = random.random()
            if (x<0.3):
                arr[i][j] = -1
            elif x<0.53333:
                arr[i][j] = 1
            elif x<0.76666:
                arr[i][j] = 2
            else:
                arr[i][j] = 3
    
    while True:
        ex = random.randint(0,dim-1)
        ey = random.randint(0,dim-1)
        if arr[ex,ey] != -1:
            break
        
       
    return arr, ex, ey




#arr, ex, ey = create_gridworld(10)


'''
ctrblocked = 0
ctr1 = 0
ctr2 = 0
ctr3 = 0

arr, ex, ey = create_gridworld(100)

for i in range(100):
    for j in range(100):
        if arr[i][j] == -1:
            ctrblocked+=1
        elif arr[i][j] == 1:
            ctr1+=1
        elif arr[i][j] == 2:
            ctr2+=1
        elif arr[i][j] == 3:
            ctr3+=1
'''