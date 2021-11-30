'''

@author: rohit-kumar-j 
@date and time :11/28/2021 1:20:32 AM IST
@description: basic utilities for debugging stuff in pybullet

'''
import pybullet as p
import pybullet_data
import time
import math
from tabulate import tabulate

class Debug(object): 
	'''
	axis/visual/collision/darken_linsks/3d_line debug tools
	'''
	def __init__(self, pybullet_client, robot_id):
		self._pybullet_client = pybullet_client
		self._robot_id = robot_id
		self._translucent = [0, 0, 0, 0.04]
		self._com_rgb =[0.0,0.0,0.0] 
		self.revert_colors_back_to_normal(capture_color=True,revert_color=False)
		# self.all_links_translucent()
	
	def robot_idx_data(self,ready_joints):
		'''
		Args:
			Pass in a string of ready joints 
		Returns:
			prints a table of joint/link name/idx
		'''
		joint_idx = []
		joint_name = []
		link_name = []
		link_idx = []
		num_j = self._pybullet_client.getNumJoints(self._robot_id)
		temp_1 = [0]*num_j
		# temp_1.append(ready_joints.sort())
		for i in range(self._pybullet_client.getNumJoints(self._robot_id)):
			temp = self._pybullet_client.getJointInfo(self._robot_id,i)
			joint_name.append(temp[1])
			joint_idx.append(temp[0])
			link_name.append(temp[12])
			link_idx.append(temp[16])
			for j in range(len(ready_joints)):
				if ready_joints[j] == joint_idx[i]:
					temp_1[i] = "Ready"
		cols = zip(temp_1,joint_idx,joint_name,link_idx,link_name)
		headers = ["ready_joints","joint_idx","joint_name","parent_link_idx","parent_link_name"]
		print(tabulate(cols,headers, tablefmt="presto"))

	def all_links_translucent(self):
		'''
		Make all links translucent rgba: [0, 0, 0, 0.05]
		'''
		for k in range(-1,self._pybullet_client.getNumJoints(self._robot_id)+1):
			self._pybullet_client.changeVisualShape(self._robot_id, k, rgbaColor=self._translucent)

	def darken_selected_links(self,links,rgb_list=None):
		'''
		Darkens the selected links to given rgb list
		If no rgb list is provided, then it defaults to self._translucent
		'''
		if rgb_list==None:
			for i in range(len(links)):
				rgb_list = self._translucent
		for i in range(len(links)):
			self._pybullet_client.changeVisualShape(self._robot_id,links[i],rgbaColor=rgb_list)
	def revert_colors_back_to_normal(self,capture_color= False,revert_color=False):
		'''
		Samples and reverts back colors to original if needed
		'''
		if capture_color == True:
			self._robot_rgba = []
			for i in range(self._pybullet_client.getNumJoints(self._robot_id)):
				rgba = self._pybullet_client.getVisualShapeData(self._robot_id)[i][7]
				self._robot_rgba.append(rgba)
		if revert_color == True:
			for i in range(self._pybullet_client.getNumJoints(self._robot_id)):
				self._pybullet_client.changeVisualShape(self._robot_id,
														i-1,
														rgbaColor = self._robot_rgba[i])

	def show_origin(self, origin_list, length=0.08,lifeTime=1.0):
		'''
		Shows origin via getLinkState[4th idx] function and Link name with getJointInfo[12th idx]
		'''	
		for i in range(len(origin_list)):
			origin = self._pybullet_client.getLinkState(self._robot_id, origin_list[i])[4]
			local_orn = self._pybullet_client.getLinkState(self._robot_id, origin_list[i])[5]
			reorder_orn = [local_orn[3],local_orn[0],local_orn[1],local_orn[2]]
			pos_x = self.quat_vec_mul(reorder_orn,[0.1,0.0,0.0])
			pos_y = self.quat_vec_mul(reorder_orn,[0.0,0.1,0.0])
			pos_z = self.quat_vec_mul(reorder_orn,[0.0,0.0,0.1])
			# print(origin)
			origin_x = (origin[0]+pos_x[0],origin[1]+pos_x[1],origin[2]+pos_x[2])
			origin_y = (origin[0]+pos_y[0],origin[1]+pos_y[1],origin[2]+pos_y[2])
			origin_z = (origin[0]+pos_z[0],origin[1]+pos_z[1],origin[2]+pos_z[2])

			# origin_x = [origin[0]+origin_x0[0],origin[1]+origin_x0[1],origin[2]+origin_x0[2]]
			# origin_y = [origin[0]+origin_y0[0],origin[1]+origin_y0[1],origin[2]+origin_y0[2]]
			# origin_z = [origin[0]+origin_z0[0],origin[1]+origin_z0[1],origin[2]+origin_z0[2]]

			self._pybullet_client.addUserDebugLine(origin, 
													origin_x,
													lineColorRGB=[1.0, 0.0, 0.0],
													lineWidth=2,
													lifeTime=lifeTime)
			self._pybullet_client.addUserDebugLine(origin, 
													origin_y,
													lineColorRGB=[0.0, 1.0, 0.0],
													lineWidth=2,
													lifeTime=lifeTime)
			self._pybullet_client.addUserDebugLine(origin, 
													origin_z,
													lineColorRGB=[0.0, 0.0, 1.0],
													lineWidth=2,
													lifeTime=lifeTime)
			self._pybullet_client.addUserDebugText(text=str(self._pybullet_client.getJointInfo(self._robot_id, int(origin_list[i]))[12]),
													 textPosition=origin,
													 textColorRGB=[0.0, 0.0, 0.0],
													 textSize=1,
													 lifeTime=lifeTime)

	def show_point(self, point, my_strings="No Text Specified", length = 0.05, lifeTime=0.0):
		'''
		Shows point and writes text
			Pass in point as list: point = [x,y,z]
		'''
		origin_x = (point[0]+length, point[1], point[2])
		origin_y = (point[0], point[1]+length, point[2])
		origin_z = (point[0], point[1], point[2]+length)

		origin_x1 = (point[0]-length, point[1], point[2])
		origin_y1 = (point[0], point[1]-length, point[2])
		origin_z1 = (point[0], point[1], point[2]-length)	

		self._pybullet_client.addUserDebugLine(origin_x1, 
							origin_x,
							lineColorRGB=[1.0, 0.0, 0.0],
							lineWidth=2,
							lifeTime=lifeTime)
		self._pybullet_client.addUserDebugLine(origin_y1, 
							origin_y,
							lineColorRGB=[0.0, 1.0, 0.0],
							lineWidth=2,
							lifeTime=lifeTime)
		self._pybullet_client.addUserDebugLine(origin_z1, 
							origin_z,
							lineColorRGB=[0.0, 0.0, 1.0],
							lineWidth=2,
							lifeTime=lifeTime)
		self._pybullet_client.addUserDebugText(text=str(my_strings),
						 textPosition=point,
						 textColorRGB=[0.0, 0.0, 0.0],
						 textSize=1,
						 lifeTime=lifeTime)

	def show_com(self, link_list,lifeTime=None,length=0.025):
		'''
		Shows the COM of the specified Link and prints the "link_name"+"...COM..."
		com_name = com name
		'''
		for i in range(len(link_list)):
			centerOfMass = self._pybullet_client.getLinkState(self._robot_id,link_list[i])[0]
			com_name = str(self._pybullet_client.getJointInfo(self._robot_id,link_list[i])[12])
			my_strings=str(com_name)+"...COM..."

			local_orn = self._pybullet_client.getLinkState(self._robot_id, link_list[i])[1]
			reorder_orn = [local_orn[3],local_orn[0],local_orn[1],local_orn[2]]
			pos_x = self.quat_vec_mul(reorder_orn,[length,0.0,0.0])
			pos_y = self.quat_vec_mul(reorder_orn,[0.0,length,0.0])
			pos_z = self.quat_vec_mul(reorder_orn,[0.0,0.0,length])

			pos_x1 = self.quat_vec_mul(reorder_orn,[-length,0.0,0.0])
			pos_y1 = self.quat_vec_mul(reorder_orn,[0.0,-length,0.0])
			pos_z1 = self.quat_vec_mul(reorder_orn,[0.0,0.0,-length])

			origin_x = (centerOfMass[0]+pos_x[0], centerOfMass[1]+pos_x[1], centerOfMass[2]+pos_x[2])
			origin_y = (centerOfMass[0]+pos_y[0], centerOfMass[1]+pos_y[1], centerOfMass[2]+pos_y[2])
			origin_z = (centerOfMass[0]+pos_z[0], centerOfMass[1]+pos_z[1], centerOfMass[2]+pos_z[2])

			origin_x0 = (centerOfMass[0]+pos_x1[0], centerOfMass[1]+pos_x1[1], centerOfMass[2]+pos_x1[2])
			origin_y0 = (centerOfMass[0]+pos_y1[0], centerOfMass[1]+pos_y1[1], centerOfMass[2]+pos_y1[2])
			origin_z0 = (centerOfMass[0]+pos_z1[0], centerOfMass[1]+pos_z1[1], centerOfMass[2]+pos_z1[2])

			self._pybullet_client.addUserDebugLine(origin_x0, 
								origin_x,
								lineColorRGB=self._com_rgb,
								lineWidth=3,
								lifeTime=lifeTime)
			self._pybullet_client.addUserDebugLine(origin_y0, 
								origin_y,
								lineColorRGB=self._com_rgb,
								lineWidth=3,
								lifeTime=lifeTime)
			self._pybullet_client.addUserDebugLine(origin_z0, 
								origin_z,
								lineColorRGB=self._com_rgb,
								lineWidth=3,
								lifeTime=lifeTime)
			self._pybullet_client.addUserDebugText(text=str(my_strings),
							 textPosition=centerOfMass,
							 textColorRGB=[0.0, 0.0, 0.0],
							 textSize=1,
							 lifeTime=lifeTime)

	def show_joint_axis(self,joint_list,scale=1.0,lifeTime=1.0):
		'''
		Shows joint axis + origin + name
		'''
		for i in range(len(joint_list)):
			#parent link index, -1 for base
			parent_link_idx = self._pybullet_client.getJointInfo(self._robot_id,joint_list[i])[16]
			
			#world position of the URDF link frame
			parent_rel_wrld = self._pybullet_client.getLinkState(self._robot_id,parent_link_idx+1)[4]
			
			#joint axis in local frame (ignored for JOINT_FIXED)
			joint_axis = self._pybullet_client.getJointInfo(self._robot_id,joint_list[i])[13]
			
			#Cartesian orientation of center of mass, in quaternion [x,y,z,w]
			par_link_orn = self._pybullet_client.getLinkState(self._robot_id,joint_list[i])[1]
			reorder_orn = [par_link_orn[3],par_link_orn[0],par_link_orn[1],par_link_orn[2]]
			
			origin = [parent_rel_wrld[0],
						parent_rel_wrld[1],
						parent_rel_wrld[2]]
			pos = self.return_long_joint_axis(joint_axis,scale = scale)
			pos_x = self.quat_vec_mul(reorder_orn, pos[0]) 
			pos_y = self.quat_vec_mul(reorder_orn, pos[1])
			pos_z = self.quat_vec_mul(reorder_orn, pos[2])

			origin_x0  =[pos_x[0]+origin[0],pos_x[1]+origin[1],pos_x[2]+origin[2]]
			origin_y0  =[pos_y[0]+origin[0],pos_y[1]+origin[1],pos_y[2]+origin[2]]
			origin_z0  =[pos_z[0]+origin[0],pos_z[1]+origin[1],pos_z[2]+origin[2]]

			origin_x1  =[origin[0]-pos_x[0],origin[1]-pos_x[1],origin[2]-pos_x[2]]
			origin_y1  =[origin[0]-pos_y[0],origin[1]-pos_y[1],origin[2]-pos_y[2]]
			origin_z1  =[origin[0]-pos_z[0],origin[1]-pos_z[1],origin[2]-pos_z[2]]

			my_strings = str(self._pybullet_client.getJointInfo(self._robot_id, int(joint_list[i]))[1])+"...Axis..."

			self._pybullet_client.addUserDebugLine(origin_x1, 
													origin_x0,
													lineColorRGB=[0.2, 0.0, 0.0],
													lineWidth=2,
													lifeTime=lifeTime)
			self._pybullet_client.addUserDebugLine(origin_y1, 
													origin_y0,
													lineColorRGB=[0.2, 0.0, 0.0],
													lineWidth=2,
													lifeTime=lifeTime)
			self._pybullet_client.addUserDebugLine(origin_z1, 
													origin_z0,
													lineColorRGB=[0.2, 0.0, 0.0],
													lineWidth=2,
													lifeTime=lifeTime)
			self._pybullet_client.addUserDebugText(text=str(my_strings),
													 textPosition=origin,
													 textColorRGB=[0.0, 0.0, 0.0],
													 textSize=1,
													 lifeTime=lifeTime)

	def swing_all(self,prismatic=None,revolute=None,stepSim=False,color_moving_links=False):
		'''
		Moves all joints to max and then min positions and the xeros them out	
		'''
		prismatic_flag = False
		revolute_flag = False
		# print("stepSim = ",stepSim)
		print("\n")
		print("================================","\n")
		for i in range(self._pybullet_client.getNumJoints(self._robot_id)):
			joint_name = self._pybullet_client.getJointInfo(self._robot_id,i)[1]
			joint_type = self._pybullet_client.getJointInfo(self._robot_id,i)[2]
			joint_min= self._pybullet_client.getJointInfo(self._robot_id,i)[8]
			joint_max= self._pybullet_client.getJointInfo(self._robot_id,i)[9]
			print("joint_name = ",joint_name)
			if joint_type == 0:
				print("joint_type = REVOLUTE")
				print("joint_min = ",  joint_min)
				print("joint_max = ",  joint_max)
				if stepSim==True:
					self.oscillate_revolute(i, 0.05,stepSim=True)
				elif stepSim==False:
					self.oscillate_revolute(i, 0.05,stepSim=False)
			elif joint_type == 1:
				print("joint_type = PRISMATIC")
				print("joint_min = ",  joint_min)
				print("joint_max = ",  joint_max)
				if stepSim==True:
					self.oscillate_prismatic(i, 0.05,stepSim=True)
				elif stepSim==False:
					self.oscillate_prismatic(i, 0.05,stepSim=False)
			elif joint_type == 4:
				print("joint_type = RIGID/FIXED")
			print("\n")
		print("================================")
		self.revert_colors_back_to_normal(revert_color=True)
			
	def oscillate_prismatic(self,joint_idx,speed,stepSim,color_moving_links=False):
		'''
		Checks if (prismatic jonit) > darkens > oscillates > Zeros > lightens
		'''
		prismatic_flag = False
		joint_type = self._pybullet_client.getJointInfo(self._robot_id,joint_idx)[2]
		assert joint_type == 1, "Joint is not prismatic" #1 == JOINT_PRISMATIC
		joint_min= self._pybullet_client.getJointInfo(self._robot_id,joint_idx)[8]
		joint_max= self._pybullet_client.getJointInfo(self._robot_id,joint_idx)[9]
		k = 0.0
		move = speed
		# print("pris_flag!= True")
		if color_moving_links == True:
			self.darken_selected_links([joint_idx],rgb_list=[0.0,0.0,0.0,0.7])
		while prismatic_flag!=True:
			k=k+move
			if k > joint_max:
				move = -move
				k=k+move
				self._pybullet_client.setJointMotorControl2(self._robot_id, 
															joint_idx, 
															controlMode=self._pybullet_client.POSITION_CONTROL, 
															targetPosition=k, 
															force=500)
			elif k<joint_max and k>joint_min:
				k=k+move
				self._pybullet_client.setJointMotorControl2(self._robot_id, 
															joint_idx, 
															controlMode=self._pybullet_client.POSITION_CONTROL, 
															targetPosition=k, 
															force=500)	
			elif k < joint_min:
				move = -move
				k=k+move
				self._pybullet_client.setJointMotorControl2(self._robot_id, 
															joint_idx, 
															controlMode=self._pybullet_client.POSITION_CONTROL, 
															targetPosition=k, 
															force=500)
				prismatic_flag = True
			# print("k = ",k)
			if stepSim ==True:
				# print("stepSim = ",stepSim)
				self._pybullet_client.stepSimulation()
				time.sleep(1./240.)
		if color_moving_links == True:
			self.darken_selected_links([joint_idx])
		if prismatic_flag == True:
			self._pybullet_client.setJointMotorControl2(self._robot_id, 
														joint_idx, 
														controlMode=self._pybullet_client.POSITION_CONTROL, 
														targetPosition=0.0, 
														force=500)
			if stepSim == True:
				# print("stepSim = ",stepSim)
				self._pybullet_client.stepSimulation()
				time.sleep(1./240.)

	def oscillate_revolute(self,joint_idx,speed,stepSim,color_moving_links=False):
		'''
		Checks if (prismatic jonit) > darkens > oscillates > Zeros > lightens
		'''
		revolute_flag = False
		joint_type = self._pybullet_client.getJointInfo(self._robot_id,joint_idx)[2]
		assert joint_type == 0, "Joint is not revolute" #1 == JOINT_REVOLUTE
		joint_min= self._pybullet_client.getJointInfo(self._robot_id,joint_idx)[8]
		joint_max= self._pybullet_client.getJointInfo(self._robot_id,joint_idx)[9]
		
		k = 0.0
		move = speed
		# print("pris_flag!= True")
		if color_moving_links == True:
			self.darken_selected_links([joint_idx],rgb_list=[0.0,0.0,0.0,0.7])
		while revolute_flag!=True:
			if k > joint_max:
				move = -move
				k=k+move
				self._pybullet_client.setJointMotorControl2(self._robot_id, 
															joint_idx, 
															controlMode=self._pybullet_client.POSITION_CONTROL, 
															targetPosition=k, 
															force=500)
			elif k<joint_max and k>joint_min:
				k=k+move
				self._pybullet_client.setJointMotorControl2(self._robot_id, 
															joint_idx, 
															controlMode=self._pybullet_client.POSITION_CONTROL, 
															targetPosition=k, 
															force=500)		
			elif k < joint_min:
				move = -move
				k=k+move
				self._pybullet_client.setJointMotorControl2(self._robot_id, 
															joint_idx, 
															controlMode=self._pybullet_client.POSITION_CONTROL, 
															targetPosition=k, 
															force=500)
				revolute_flag = True

			# print("k = ",k)
			if stepSim==True:
				# print("stepSim = ",stepSim)
				self._pybullet_client.stepSimulation()
				time.sleep(1./240.)
		if color_moving_links == True:
			self.darken_selected_links([joint_idx])
		if revolute_flag == True:
			self._pybullet_client.setJointMotorControl2(self._robot_id, 
														joint_idx, 
														controlMode=self._pybullet_client.POSITION_CONTROL, 
														targetPosition=0.0, 
														force=500)
			if stepSim==True:
				# print("stepSim = ",stepSim)
				self._pybullet_client.stepSimulation()
				time.sleep(1./240.)

	###### Utilities ######

	def q_conjugate(self,q):
		return (q[0], -q[1], -q[2], -q[3])

	def quat_vec_mul(self,q1, v1):
		q2 = [0.0,] + v1
		return self.quat_mul(self.quat_mul(q1, q2), self.q_conjugate(q1))[1:]

	def quat_mul(self,q1, q2):
		w = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]
		x = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]
		y = q1[0] * q2[2] + q1[2] * q2[0] + q1[3] * q2[1] - q1[1] * q2[3]
		z = q1[0] * q2[3] + q1[3] * q2[0] + q1[1] * q2[2] - q1[2] * q2[1]
		return [w, x, y, z]

	def return_long_joint_axis(self,joint_axis,scale = 1.0):
		'''
		Simply checks for the largest absolute value and returns the posx,y,z additions
		'''
		for i in range(1, len(joint_axis)+1):
			if abs(joint_axis[0])> abs(joint_axis[1]) and abs(joint_axis[0])>abs(joint_axis[2]):
				axis = [[scale+0.05,0.0,0.0],[0.0,0.01,0.0],[0.0,0.0,0.01]]
			elif abs(joint_axis[1])> abs(joint_axis[0]) and abs(joint_axis[1])>abs(joint_axis[2]):
				axis = [[0.01,0.0,0.0],[0.0,scale+0.05,0.0],[0.0,0.0,0.01]]
			elif abs(joint_axis[2])> abs(joint_axis[0]) and abs(joint_axis[2])>abs(joint_axis[1]):
				axis = [[0.01,0.0,0.0],[0.0,0.01,0.0],[0.0,0.0,scale+0.05]]
			return axis

	def basic_rendering(self,shadows=True,wireframe=False,gui=False):
		'''
		Add flags for removing the configVisualDebugger
		'''
		if shadows==False:
			self._pybullet_client.configureDebugVisualizer(self._pybullet_client.COV_ENABLE_SHADOWS,0)
		if shadows==True:
			self._pybullet_client.configureDebugVisualizer(self._pybullet_client.COV_ENABLE_SHADOWS,1)
		if gui==False:
			self._pybullet_client.configureDebugVisualizer(self._pybullet_client.COV_ENABLE_GUI,0)
		if gui==True:
			self._pybullet_client.configureDebugVisualizer(self._pybullet_client.COV_ENABLE_GUI,1)
		if wireframe==False:
			self._pybullet_client.configureDebugVisualizer(self._pybullet_client.COV_ENABLE_WIREFRAME,0)
		if wireframe==True:
			self._pybullet_client.configureDebugVisualizer(self._pybullet_client.COV_ENABLE_WIREFRAME,1)