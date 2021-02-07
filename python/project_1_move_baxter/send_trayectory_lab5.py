#!/usr/bin/env python
import rospy
from baxter_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
import Make_trajectory as mt
import numpy as np

def callback(angles):
	global datostheta
	dof_c = angles.position
	datostheta = {'r':[dof_c[12],dof_c[13],dof_c[14],dof_c[15],dof_c[16],dof_c[17],dof_c[18]],\
		          'l':[dof_c[3],dof_c[4],dof_c[5],dof_c[6],dof_c[7],dof_c[8],dof_c[9]]}


def trayectory(dof):
	# Iniciamos nodo principal
	rospy.init_node('trayectory', anonymous=True)
	# Left Arm
	s_traj_left = rospy.Publisher('robot/limb/left/joint_command', JointCommand, queue_size=10)
	rospy.Subscriber('/robot/joint_states', JointState, callback)
	dof_baxter_left = JointCommand()
	dof_baxter_left.mode = 1
	dof_baxter_left.command = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
	dof_baxter_left.names = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2']

	# Left Arm
	s_traj_right = rospy.Publisher('robot/limb/right/joint_command', JointCommand, queue_size=10)
	dof_baxter_right = JointCommand()
	dof_baxter_right.mode = 1
	dof_baxter_right.command = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
	dof_baxter_right.names = ['right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']
	rate = rospy.Rate(50)
	
	# Iniciamos el contador
	k = 0 # Left counter
	w = 0 # Right counter
	
	while not rospy.is_shutdown():
		njc_left = np.array([dof['l'][0,k],dof['l'][1,k],0.0,dof['l'][2,k],\
			                  dof['l'][3,k],dof['l'][4,k],dof['l'][5,k]])
		njc_right = np.array([dof['r'][0,w],dof['r'][1,w],0.0,dof['r'][2,w],\
			                  dof['r'][3,w],dof['r'][4,w],dof['r'][5,w]])
		dv_left = np.absolute(datostheta['l']-njc_left)
		dv_right = np.absolute(datostheta['r']-njc_right)
		
		if (dv_left[0]<0.01 or dv_left[1]<0.01 or dv_left[2]<0.01 or dv_left[3]<0.01 or dv_left[4]<0.01\
			 or dv_left[5]<0.01 or dv_left[6]<0.01):
			k = k + 1
	
		if k == 800:
			k = k -1
		if (dv_right[0]<0.01 or dv_right[1]<0.01 or dv_right[2]<0.01 or dv_right[3]<0.01 or dv_right[4]<0.01\
			 or dv_right[5]<0.01 or dv_right[6]<0.01):
			w = w + 1
			
		if w == 800:
			w = w -1
		# Enviamos comandos 
		dof_baxter_right.command = njc_right
		s_traj_right.publish(dof_baxter_right)
		dof_baxter_left.command = njc_left
		s_traj_left.publish(dof_baxter_left)
		rate.sleep()
		

if __name__ == '__main__':
	try:
		# Inicializamos variables asociada a los dof_current
		datostheta = {'r':[0,0,0,0,0,0,0],'l':[0,0,0,0,0,0,0]}

		# Definimos condiciones trajectorias
		# Trayectoria 1
		p0_r1 = [-1,-0.5,1.4]
		pf_r1 = [-1.2,-0.5,1]
		p0_l1 = [1.2,-0.5,0.9]
		pf_l1 = [1.2,-0.5,1.1]
		or0_r1 = [0.7854,1.5708,-1.5708]
		orf_r1 = [0.7854,1.5708,-1.5708]
		or0_l1 = [0.7854,1.5708,0]
		orf_l1 = [0.7854,1.5708,0]
		# Trayectoria 2
		p0_r2 = [-1.2,-0.5,1]
		pf_r2 = [-0.8,-0.5,1]
		p0_l2 = [1.2,-0.5,1.1]
		pf_l2 = [0.8,-0.5,1.1]
		or0_r2 = [0.7854,1.5708,-1.5708]
		orf_r2 = [0.7854,1.5708,-1.5708]
		or0_l2 = [0.7854,1.5708,0]
		orf_l2 = [0.7854,1.5708,0]
		# Trayectoria 3
		p0_r3 = [-0.8,-0.5,1]
		pf_r3 = [-1,-0.5,1.4]
		p0_l3 = [0.8,-0.5,1.1]
		pf_l3 = [0.8,-0.5,0.9]
		or0_r3 = [0.7854,1.5708,-1.5708]
		orf_r3 = [0.7854,1.5708,-1.5708]
		or0_l3 = [0.7854,1.5708,0]
		orf_l3 = [0.7854,1.5708,0]
		# Trayectoria 4
		p0_r4 = [-1,-0.5,1.4]
		pf_r4 = [-1.2,-0.5,1]
		p0_l4 = [0.8,-0.5,0.9]
		pf_l4 = [1.2,-0.5,0.9]
		or0_r4 = [0.7854,1.5708,-1.5708]
		orf_r4 = [0.7854,1.5708,-1.5708]
		or0_l4 = [0.7854,1.5708,0]
		orf_l4 = [0.7854,1.5708,0]
		# Generamos los vectores catesianos 
		v_cart1 = mt.dof_trajectory(p0_r1,p0_l1,pf_r1,pf_l1,or0_r1,or0_l1,orf_r1,orf_l1)
		v_cart2 = mt.dof_trajectory(p0_r2,p0_l2,pf_r2,pf_l2,or0_r2,or0_l2,orf_r2,orf_l2)
		v_cart3 = mt.dof_trajectory(p0_r3,p0_l3,pf_r3,pf_l3,or0_r3,or0_l3,orf_r3,orf_l3)
		v_cart4 = mt.dof_trajectory(p0_r4,p0_l4,pf_r4,pf_l4,or0_r4,or0_l4,orf_r4,orf_l4)
		# Concatenamos vector total
		v_cart_p = {'r':np.concatenate((v_cart1['pose']['r'], v_cart2['pose']['r']), axis=1),\
			        'l':np.concatenate((v_cart1['pose']['l'], v_cart2['pose']['l']), axis=1)}
		v_cart_p = {'r':np.concatenate((v_cart_p['r'], v_cart3['pose']['r']), axis=1),\
			        'l':np.concatenate((v_cart_p['l'], v_cart3['pose']['l']), axis=1)}
		v_cart_p = {'r':np.concatenate((v_cart_p['r'], v_cart4['pose']['r']), axis=1),\
			        'l':np.concatenate((v_cart_p['l'], v_cart4['pose']['l']), axis=1)}
		
		v_cart_a = {'r':np.concatenate((v_cart1['orien']['r'], v_cart2['orien']['r']), axis=1),\
			        'l':np.concatenate((v_cart1['orien']['l'], v_cart2['orien']['l']), axis=1)}
		v_cart_a = {'r':np.concatenate((v_cart_a['r'], v_cart3['orien']['r']), axis=1),\
			        'l':np.concatenate((v_cart_a['l'], v_cart3['orien']['l']), axis=1)}
		v_cart_a = {'r':np.concatenate((v_cart_a['r'], v_cart4['orien']['r']), axis=1),\
			        'l':np.concatenate((v_cart_a['l'], v_cart4['orien']['l']), axis=1)}

		v_cart = {'pose':v_cart_p,'orien':v_cart_a}

		# Generamos los vectores articulares
		v_dof = mt.dof_thetas(v_cart,800)

		# Mandamos los valores articulares al robot
		trayectory(v_dof)
	except rospy.ROSInterruptException:
		pass


