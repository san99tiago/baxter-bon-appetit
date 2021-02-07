#!/usr/bin/env python
# Forward Kinematic (Baxter Melaje)
# Importamos modulos importantes 
import numpy as np
import math
import TransformationMatrix as tm
import DH as dh

# Definimos la funcion para calcular la TPK
def FPK(dof,ds,arm,ndof):
    ## Definimos cantidades importantes
    # Tabla de parametros DH brazo baxter 7dof
    DHTable0_7 = np.array([[0,0,0,dof[0]],\
                 [-math.pi/2,ds[1],0,(dof[1]+math.pi/2)],\
                 [math.pi/2,0,ds[2],dof[2]],\
                 [-math.pi/2,ds[3],0,dof[3]],\
                 [math.pi/2,0,ds[4],dof[4]],\
                 [-math.pi/2,ds[5],0,dof[5]],\
                 [math.pi/2,0,0,dof[6]]])
    # Tabla de parametros DH brazo baxter 6dof
    DHTable0_6 = np.array([[0,0,0,dof[0]],\
                 [-math.pi/2,ds[1],0,dof[1]],\
                 [0,math.sqrt(ds[2]**2 + ds[3]**2),0,(dof[3]+math.pi/2)],\
                 [math.pi/2,0,ds[4],dof[4]],\
                 [-math.pi/2,ds[5],0,dof[5]],\
                 [math.pi/2,0,0,dof[6]]])
    # Matriz de transformacion wo->bl
    Two_bl = tm.transformation_matrix('z',-math.pi/4,[ds[7],-ds[8],ds[9]])
    # Matriz de transformacion wo->br
    Two_br = tm.transformation_matrix('z',-math.pi*3/4,[-ds[7],-ds[8],ds[9]])
    # Matriz de transformacion bl/br->0
    Tb_0 = tm.transformation_matrix('z',0,[0,0,ds[0]])
    # Matriz de transformacion 7->gt
    T7_gt = tm.transformation_matrix('z',0,[0,0,ds[6]])
    
    ## Calculamos la TPK depio del brazo 
    if ndof == 6:
        if arm == 'r':
            Two_gt = np.dot(np.dot(np.dot(Two_br,Tb_0),dh.directKinematic(DHTable0_6,False)),T7_gt)
        if arm == 'l':
            Two_gt = np.dot(np.dot(np.dot(Two_bl,Tb_0),dh.directKinematic(DHTable0_6,False)),T7_gt)
    if ndof == 7:
        if arm == 'r':
            Two_gt = np.dot(np.dot(np.dot(Two_br,Tb_0),dh.directKinematic(DHTable0_7,False)),T7_gt)
        if arm == 'l':
            Two_gt = np.dot(np.dot(np.dot(Two_bl,Tb_0),dh.directKinematic(DHTable0_7,False)),T7_gt)
    
    # Retornamos la matriz solucion 
    return Two_gt
            
if __name__ == '__main__':
    pass
    
