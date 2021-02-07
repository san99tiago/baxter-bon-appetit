#!/usr/bin/env python
# DH 
# Importamos modulos importantes 
import numpy as np
import math
import TransformationMatrix as tm

def directKinematic(TableOfParams, type):
    """
       En esta funcion se calcula la cinematica directa de cualquier robot,
       tiene como entrada la tabla de parametros de DH del robot.
       Es posible escoger trabajar con grados y con radianes 
       Si type == true se trabaja en grados y si type == false en radianes
    """
    
    # Primero obtenemos el numero de filas de la matriz de parametros 
    n = len(TableOfParams)
    
    # Definimos si trabajamos en grados o en radianes 
    if type:
        for i in range(n):
            TableOfParams[i,1] = math.radians(TableOfParams[i,0])
            TableOfParams[i,4] = math.radians(TableOfParams[i,3])
        
     
    
    # Definimos una matriz auxiliar donde se acumularan los productos 
    AUX = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    
    # Realizamos los calculos necesarios
    for j in range(n):
        # Calculamos la rotacion de alpha de i-1
        TMalpha = tm.transformation_matrix('x',TableOfParams[j,0],[0,0,0])
        # Calculamos la traslacion de a de i-1
        TMa = tm.transformation_matrix('z',0,[TableOfParams[j,1],0,0])
        # Calculamos la traslacion de b de i
        TMb = tm.transformation_matrix('z',0,[0,0,TableOfParams[j,2]])
        # Calculamos la rotacion de theta de i
        TMtheta = tm.transformation_matrix('z',TableOfParams[j,3],[0,0,0])
        # Calculamos la matriz de transformacion total
        TMtotal = np.dot(np.dot(np.dot(TMalpha,TMa),TMb),TMtheta)

        # Calculamos el acumulado de las matrices de transformacion 
        AUX = np.dot(AUX,TMtotal)
    
    # Definimos la matriz de transformacion de la cinematica directa
    DC_TMatrix_Out = AUX

    # Retornamos la matriz solucion 
    return DC_TMatrix_Out

if __name__ == '__main__':
    pass
    