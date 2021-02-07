#!/usr/bin/env python
# Make trajectory
# Importamos modulos importantes 
import numpy as np
import math
import trajectory_5order as t5o
import Rotation_Matrix_expand as re
import BaxterClass as bc

# Definimos la funcion asociada a construir los vectores de la trayectoria
def dof_trajectory(p0_r,p0_l,pf_r,pf_l,or0_r,or0_l,orf_r,orf_l): 
    # Definimos los valores asociados al tiempo de la simulacion 
    tf = 5
    tp = np.linspace(0, tf, 200, endpoint=True)
    
    # Cantidades asociadas a la traslacion 
    linear = {'r':{'p0':p0_r,\
                               'pf':pf_r,\
                               'v0':[0,0,0],\
                               'vf':[0,0,0],\
                               'a0':[0,0,0],\
                               'af':[0,0,0]},\
              'l':{'p0':p0_l,\
                               'pf':pf_l,\
                               'v0':[0,0,0],\
                               'vf':[0,0,0],\
                               'a0':[0,0,0],\
                               'af':[0,0,0]}} 
    # Cantidades asociadas a la orientacion
    ori = {'r':{'p0':or0_r,\
                           'pf':orf_r,\
                           'v0':[0,0,0],\
                           'vf':[0,0,0],\
                           'a0':[0,0,0],\
                           'af':[0,0,0]},\
                'l':{'p0':or0_l,\
                           'pf':orf_l,\
                           'v0':[0,0,0],\
                           'vf':[0,0,0],\
                           'a0':[0,0,0],\
                           'af':[0,0,0]}}
    
    # Inicializamos las variables 
    p = {'r':np.array([tp*0,tp*0,tp*0]),'l':np.array([tp*0,tp*0,tp*0])}
    angles = {'r':np.array([tp*0,tp*0,tp*0]),'l':np.array([tp*0,tp*0,tp*0])}
    
    for j in range(len(tp)):
        ## Construimos los vectores asociados a las cantidades cinematicas
        for i in range(3):
            # BRAZO DERECHO
            # Cantidades traslacionales
            p['r'][i,j] = t5o.trayectoria_5_orden(tp[j],tf,linear['r']['p0'][i],\
            linear['r']['pf'][i],linear['r']['v0'][i],linear['r']['vf'][i],linear['r']['a0'][i],linear['r']['af'][i])
            # Cantidades de la orientacion
            angles['r'][i,j] = t5o.trayectoria_5_orden(tp[j],tf,ori['r']['p0'][i],\
            ori['r']['pf'][i],ori['r']['v0'][i],ori['r']['vf'][i],ori['r']['a0'][i],ori['r']['af'][i])
            # BRAZO IZQUIERDO
            # Cantidades traslacionales
            p['l'][i,j] = t5o.trayectoria_5_orden(tp[j],tf,linear['l']['p0'][i],\
            linear['l']['pf'][i],linear['l']['v0'][i],linear['l']['vf'][i],linear['l']['a0'][i],linear['l']['af'][i])
            # Cantidades de la orientacion
            angles['l'][i,j] = t5o.trayectoria_5_orden(tp[j],tf,ori['l']['p0'][i],\
            ori['l']['pf'][i],ori['l']['v0'][i],ori['l']['vf'][i],ori['l']['a0'][i],ori['l']['af'][i])

    poan = {'pose':p,'orien':angles}
    return poan

def dof_thetas(vec_cart,long):
    # Creamos un objeto baxter
    b1 = bc.baxterClass()
    # Inicializamos los vectores para los grados de libertad
    dof = {'r':np.array([np.zeros(long),np.zeros(long),np.zeros(long),np.zeros(long),np.zeros(long),np.zeros(long)]),\
           'l':np.array([np.zeros(long),np.zeros(long),np.zeros(long),np.zeros(long),np.zeros(long),np.zeros(long)])}
    # Iniciamos el ciclo para llenar los vectores
    for i in range(long):
        # Encontramos la matriz de rotacion 
        R_M = {'r':re.rotation_matrix_expand([vec_cart['orien']['r'][0,i],vec_cart['orien']['r'][1,i],vec_cart['orien']['r'][2,i]]),\
               'l':re.rotation_matrix_expand([vec_cart['orien']['l'][0,i],vec_cart['orien']['l'][1,i],vec_cart['orien']['l'][2,i]])}
        Tm_full = {'r':np.array([[R_M['r'][0,0],R_M['r'][0,1],R_M['r'][0,2],vec_cart['pose']['r'][0,i]],\
                                 [R_M['r'][1,0],R_M['r'][1,1],R_M['r'][1,2],vec_cart['pose']['r'][1,i]],\
                                 [R_M['r'][2,0],R_M['r'][2,1],R_M['r'][2,2],vec_cart['pose']['r'][2,i]],\
                                 [0,0,0,1]]),\
                   'l':np.array([[R_M['l'][0,0],R_M['l'][0,1],R_M['l'][0,2],vec_cart['pose']['l'][0,i]],\
                                 [R_M['l'][1,0],R_M['l'][1,1],R_M['l'][1,2],vec_cart['pose']['l'][1,i]],\
                                 [R_M['l'][2,0],R_M['l'][2,1],R_M['l'][2,2],vec_cart['pose']['l'][2,i]],\
                                 [0,0,0,1]])}

        ipk_dof = {'r': b1.ipk(Tm_full['r'],'r','u'),'l': b1.ipk(Tm_full['l'],'l','u')}
        dof['r'][0,i] = ipk_dof['r'][0]
        dof['r'][1,i] = ipk_dof['r'][1]
        dof['r'][2,i] = ipk_dof['r'][2]
        dof['r'][3,i] = ipk_dof['r'][3]
        dof['r'][4,i] = ipk_dof['r'][4]
        dof['r'][5,i] = ipk_dof['r'][5]
        dof['l'][0,i] = ipk_dof['l'][0]
        dof['l'][1,i] = ipk_dof['l'][1]
        dof['l'][2,i] = ipk_dof['l'][2]
        dof['l'][3,i] = ipk_dof['l'][3]
        dof['l'][4,i] = ipk_dof['l'][4]
        dof['l'][5,i] = ipk_dof['l'][5]
    return dof


if __name__ == '__main__':
    a = dof_trajectory()
    print(a['pose']['l'][:,49])
    print('+++++++++++++++++++++')
    print(a['orien']['l'][:,49])
    b = dof_thetas(a,50)
    print(b['r'])

