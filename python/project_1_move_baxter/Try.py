# DH 
# Importamos modulos importantes 
import numpy as np
import math


table = np.array([[0,0,125,math.pi],[math.pi/2,225,0,math.pi/2+math.pi],[0,300,0,math.pi/4],\
                 [0,0,350,0]])

a = np.array([[1,2,3],[1,2,3],[1,2,3]])
b = np.array([[1,2,3],[1,2,3],[1,2,3]])
c = np.concatenate((a, b), axis=1)

print(c)