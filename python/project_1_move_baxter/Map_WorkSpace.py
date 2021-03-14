#!/usr/bin/env python
# Importamos modulos importantes
import rospy
from sensor_msgs.msg import JointState
import BaxterClass as bc
import sys
import numpy as np

def callback(angles):
    global datostheta
    dof_c = angles.position
    datostheta = {'r':[dof_c[11],dof_c[12],dof_c[9],dof_c[10],dof_c[13],dof_c[14],dof_c[15]],\
    	          'l':[dof_c[4],dof_c[5],dof_c[2],dof_c[3],dof_c[6],dof_c[7],dof_c[8]]}

def main():
    global cartesianPoint
    # Iniciamos nodo principal
    rospy.init_node('trayectory', anonymous=True)
    rospy.Subscriber('/robot/joint_states', JointState, callback)
    rate = rospy.Rate(50)

    # Creamos un objeto Baxter
    b1 = bc.baxterClass()

    # Abrimos el archivo donde se almacenara la informacion
    cartesianPoint = open(sys.argv[2],'a')
    point = b1.fpk(datostheta['r'],'r',7)[:3,3:4]

    while not rospy.is_shutdown():
        # Calculamos los puntos cartesianos 
        if sys.argv[1] == 'r': 
            # point = b1.fpk(datostheta['r'],'r',7)[:3,3:4]
            point = b1.fpk(datostheta['r'],'r',7)
        if sys.argv[1] == 'l':
            # point = b1.fpk(datostheta['l'],'l',7)[:3,3:4]
            point = b1.fpk(datostheta['l'],'l',7)

        print('/n')
        print(np.array2string(point))

        # Guardamos la informacion
        # savePoint = '['+ str(point[0,0])+','+str(point[1,0])+','+str(point[2,0])+']\n'
        # cartesianPoint.write(savePoint)
        # print (savePoint)
        rate.sleep()


if __name__ == '__main__':
    try:
        datostheta = {'r':[0,0,0,0,0,0,0],'l':[0,0,0,0,0,0,0]}
        main()
        cartesianPoint.close()
    except rospy.ROSInterruptException:
		print 'BAXTER ERROR'