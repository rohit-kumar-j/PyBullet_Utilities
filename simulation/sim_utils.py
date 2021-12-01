'''

@author: rohit-kumar-j 
@date and time : 2021-12-01 22:55:38
@description: basic utilities for debugging stuff in pybullet

'''
import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
os.sys.path.insert(1, parentdir)
print("currentdir = ", currentdir)
print("parentdir=", parentdir)

import time
import math
import random
import pybullet as p
import pybullet_data as pd
# from bullet_client import BulletClient
from pybullet_utils import bullet_client
from robots import panda_robot

class SimUtils(object):
    '''
    Launch Boxes, load robots, etc
    '''
    def __init__(self,bullet_client,robot_id=None):
        self._client = bullet_client

    def launch_random_boxes(self,qty,range,robot_pos):
        '''
        Args: launch boxes based on qty, size_range and robot position 
        Returns: None
        '''
        for i in range(qty):
            pos = random.random()
            box_id = self.generate_box(range)
            self.throw_box(box_id,robot_pos,i,qty)
    
    def throw_box(self,radius,client,box_id,robot_pos,curr_itr,max_itr,box_speed=10.0,speed=50.0,robot_id=None):
        '''
        draws a virtual circle around the robot and launches boxes from points on this circle
        '''
        # Get position offset from robot_base
        x = radius*math.cos((curr_itr*2*math.pi*speed)/max_itr)
        y = radius*math.sin((curr_itr*2*math.pi*speed)/max_itr)
        box_pos = [robot_pos[0] + x, robot_pos[1] + y]
        velocity_vec = [(robot_pos[0]-x)*box_speed,(robot_pos[1]-y)*box_speed,0.2]
        client.resetBasePositionAndOrientation(bodyUniqueId=box_id,
                                          posObj=[x,y,1],
                                          ornObj=[0,0,0,1])
        p.resetBaseVelocity(objectUniqueId=box_id,
                                 linearVelocity=velocity_vec) # direction * scalar
    
    def generate_box(self,size,client,pos=[0,0,10],change_color=False,rgbaColor=[1.0,0.0,0.0,1.0]):
        '''
        Generates box and returns the objectUniqueId of the box
        '''
        # if size >= 1.5: raise Exception("Too large box size!, keep size below 1.5")
        # if size <= 0.1: raise Exception("Too small box size!, keep size above 0.1")
        vuid = client.createCollisionShape(client.GEOM_BOX, halfExtents = [size,size,size])
        cuid = client.createCollisionShape(client.GEOM_BOX, halfExtents = [size,size,size])
        mb = client.createMultiBody(baseMass=size*1.5,
                                    baseInertialFramePosition=[0, 0, 0],
                                    baseCollisionShapeIndex=cuid,
                                    baseVisualShapeIndex=vuid,
                                    basePosition=pos,
                                    useMaximalCoordinates=False)
        if mb <= 0: raise Exception("creating multibody failed!")
        if change_color == True:
            p.changeVisualShape(objectUniqueId=mb,
                                linkIndex=-1,
                                rgbaColor=rgbaColor)
        return mb

# ---------------------------------- Class Tests ---------------------------------- #

def main():
    print("running directly from sim_utils...\n")
    client = bullet_client.BulletClient(connection_mode=p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    p.setGravity(0,0,-9.8)
    plane_id = p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("../robots/panda_robot/panda_arm.urdf",
                               basePosition=[0,0,0],
                               baseOrientation=[0,0,0,1],
                               useFixedBase=True,
                               globalScaling=2.0)
    sim = SimUtils(bullet_client=client,robot_id=None)
    boxes = []
    allowed_boxes = 40
    while 1:
        for i in range(100):
            if i%2 == 1:
                box_id = sim.generate_box(size=(0.2*random.random()),client=client,change_color=True,rgbaColor=[0.5,0.5,0.5,1.0])
                boxes.append(box_id)
                sim.throw_box(client=client,radius=2.0,box_id=box_id,robot_pos=[0,0,0],curr_itr=i,box_speed=40.0,max_itr=1000)
                if len(boxes) >= allowed_boxes:
                    print(len(boxes), boxes)
                    p.removeBody(boxes[0])
                    boxes=boxes[1:]
                p.stepSimulation()
                time.sleep(1./240.)

if __name__ =="__main__":
    main()