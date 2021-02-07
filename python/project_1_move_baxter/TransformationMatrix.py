#!/usr/bin/env python
# Transformation Matrix
# Importamos modulos importantes 
import numpy as np
import math

def transformation_matrix(direction,angle,position):
    # Definimos la matriz de transformacion del sistema
    if direction == 'x':
        T_Matrix_Out = np.array([[1,0,0,position[0]],[0,math.cos(angle),\
        -math.sin(angle),position[1]],[0,math.sin(angle),math.cos(angle),position[2]],[0,0,0,1]]) 
    
    if direction == 'y':
        T_Matrix_Out = np.array([[math.cos(angle),0,math.sin(angle),position[0]],[0,1,\
        0,position[1]],[-math.sin(angle),0,math.cos(angle),position[2]],[0,0,0,1]])

    if direction == 'z':
        T_Matrix_Out = np.array([[math.cos(angle),-math.sin(angle),0,position[0]],[math.sin(angle),math.cos(angle),\
        0,position[1]],[0,0,1,position[2]],[0,0,0,1]])
    
    return T_Matrix_Out
    
if __name__ == '__main__':
    pass
    
