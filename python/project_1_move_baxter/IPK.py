#!/usr/bin/env python
# Inverse Kinematic (Baxter Melaje)
# Importamos modulos importantes 
import numpy as np
import math
import cmath
import TransformationMatrix as tm

 # Definimos la funcion para calcular la IPK
def IPK(FPK,ds,arm,type):
    # Definimos cantidades importantes
    # Matriz de transformacion wo->0
    Two_0_r = np.dot(tm.transformation_matrix('z',-3*math.pi/4,[-ds[6],-ds[7],ds[8]]),\
              tm.transformation_matrix('z',0,[0,0,ds[0]]))
    Two_0_l = np.dot(tm.transformation_matrix('z',-math.pi/4,[ds[6],-ds[7],ds[8]]),\
              tm.transformation_matrix('z',0,[0,0,ds[0]]))
    
     # Matriz de transformacion 6->gt
    T6_gt = tm.transformation_matrix('z',0,[0,0,ds[5]])
    
     # Matriz de transformacion 0->6
    if arm == 'r':
        T0_6 = np.dot(np.dot(np.linalg.inv(Two_0_r),FPK),np.linalg.inv(T6_gt))
    if arm == 'l':
        T0_6 = np.dot(np.dot(np.linalg.inv(Two_0_l),FPK),np.linalg.inv(T6_gt))
        
    
     # Comenzamos calculando los dof referentes a la traslacion 
     # Calculamos theta 1
    t1 = math.atan2(T0_6[1,3],T0_6[0,3])
   
     # Calculamos theta 2
     # Definimos cantidades importantes
    lh = math.sqrt(ds[2]**2+ds[3]**2)
    E = 2*lh*(ds[1] - T0_6[0,3]/math.cos(t1))
    F = 2*lh*T0_6[2,3]
    G = (T0_6[0,3]/math.cos(t1))**2 + ds[1]**2 + lh**2 - ds[4]**2 + T0_6[2,3]**2 - \
        2*ds[1]*(T0_6[0,3]/math.cos(t1))

    a = G-E
    b = 2*F
    c = G+E
    tt21 = (-b + cmath.sqrt(b**2-4*a*c))/(2*a)
    tt22 = (-b - cmath.sqrt(b**2-4*a*c))/(2*a)

    t21 = 2*math.atan(tt21)
    t22 = 2*math.atan(tt22)
    
    if abs(t21.imag) < 2:
        t21 = t21.real
    if abs(t22.imag) < 2:
        t21 = t22.real
         
     # Calculamos theta 4
    t41 = math.atan2(-T0_6[2,3] - lh*math.sin(t21),T0_6[0,3]/math.cos(t1) - ds[1] - lh*math.cos(t21)) - t21
    t42 = math.atan2(-T0_6[2,3] - lh*math.sin(t22),T0_6[0,3]/math.cos(t1) - ds[1] - lh*math.cos(t22)) - t22
    
     # Calculamos los dof referentes a la rotacion 
     # Definimos cantidades importantes
    s1 = math.sin(t1)
    c1 = math.cos(t1)
    s24 = [math.sin(t21+t41),math.sin(t22+t42)]
    c24 = [math.cos(t21+t41),math.cos(t22+t42)]
  
    R0_3 = {'eu':np.array([[-c1*s24[0],-c1*c24[0],-s1],\
                 [-s1*s24[0],-s1*c24[0],c1],\
                 [-c24[0],s24[0],0]]),\
            'ed':np.array([[-c1*s24[1],-c1*c24[1],-s1],\
                 [-s1*s24[1],-s1*c24[1],c1],\
                 [-c24[1],s24[1],0]])}   
    R3_6 = {'eu':np.dot(R0_3['eu'].transpose(),T0_6[:3,:3]),\
            'ed':np.dot(R0_3['ed'].transpose(),T0_6[:3,:3])} 

    if type == 'u':
        # Calculamos theta 5
        t5 = math.atan2(R3_6['eu'][2,2],R3_6['eu'][0,2])
           
        # Calculamos theta 7
        t7 = math.atan2(-R3_6['eu'][1,1],R3_6['eu'][1,0])
           
        # Calculamos theta 6
        t6 = math.atan2(R3_6['eu'][1,0]/math.cos(t7),-R3_6['eu'][1,2])  

        # Definimos el vector de los dof
        vdof = [t1,t21,t41,t5,t6,t7]

    if type == 'd':
        # Calculamos theta 5
        t5 = math.atan2(R3_6['ed'][2,2],R3_6['ed'][0,2])
           
        # Calculamos theta 7
        t7 = math.atan2(-R3_6['ed'][1,1],R3_6['ed'][1,0])
           
        # Calculamos theta 6
        t6 = math.atan2(R3_6['ed'][1,0]/math.cos(t7),-R3_6['ed'][1,2])    

        # Definimos el vector de los dof
        vdof = [t1,t22,t42,t5,t6,t7]    

    # Retornamos los valores de los dof
    return vdof     
   
if __name__ == '__main__':
    pass
    
    
    
 