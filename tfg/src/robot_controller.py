#! /usr/bin/env python3

import numpy as np
import rospy
import sys, os
import time
import actionlib
import tf

from geometry_msgs.msg import Twist, PointStamped
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Clase para definir el controlador de movimiento de un robot
class robot_controller():

    # Constructor de la clase, se lanza el nodo, se crean suscriptores y publishers
    def __init__(self, ID, run_event):
        rospy.init_node("controller")

        self.point_sub = rospy.Subscriber("/clicked_point", PointStamped, self.callback_onClick)
        self.pub = rospy.Publisher("/tb3_" + str(ID) + "/cmd_vel", Twist, queue_size=1)
        self.listener = tf.TransformListener()
        rate = rospy.Rate(50)
        self.client =  actionlib.SimpleActionClient("tb3_"+str(ID)+"/move_base",MoveBaseAction)
        self.client.wait_for_server()
        
        self.run_event = run_event
        self.ID = ID
        self.clicked_point = PointStamped()
        self.home = ()

        self.set_home = True
        self.goal_achieved = False
        self.new_goal = True
        self.new_init_pose = True
        self.ready = False
        self.arrived = False

        self.init_poses = []

    def callback_onClick(self, msg):

        self.clicked_point.header.stamp = rospy.Time.now()
        self.clicked_point.header.frame_id = "map"
        self.clicked_point.point.x = msg.point.x         
        self.clicked_point.point.y = msg.point.y
        self.clicked_point.point.z = msg.point.z

        tmp = [msg.point.x, msg.point.y, msg.point.z]
        self.init_poses.append(tmp)

    def PoseDefined(self):

        return True if len(self.init_poses) == self.ID+1  else False

    def getCurrentPos(self):
        pos,rotation = self.listener.lookupTransform('map', "tb3_"+str(self.ID)+"/base_link",rospy.Time(0))
        position = round(pos[0],3), round(pos[1],3) # Tuple
        return [position, rotation]

    def setHome(self):
        if self.set_home:
            self.set_home = False
            self.home = self.getCurrentPos()

    def goToGoal(self, pose, is_home):
        
        if self.new_goal:           # Solo enviar un nuevo goal al robot cuando sea necesario (puesto que en esta función se entra constantemente, así evitamos que se sobreescriba el destino todo el tiempo)
            self.new_goal = False

            pos = pose[0]
            rot = pose[1]

            goal = MoveBaseGoal() # Type
            goal.target_pose.header.frame_id = "map" # Reference
            # if is_home:
            #     goal.target_pose.header.frame_id = "tb3_"+str(self.ID) +"/map"
            goal.target_pose.pose.position.x = pos[0]
            goal.target_pose.pose.position.y = pos[1]

            goal.target_pose.pose.orientation.z = rot[2]
            goal.target_pose.pose.orientation.w = rot[3]
            
            self.client.send_goal(goal) # Send objective point

        if self.client.get_state() is GoalStatus.SUCCEEDED:
            self.goal_achieved = True
            self.new_goal = True

    def setInitialPose(self):

        if self.new_init_pose:           # Solo enviar un nuevo goal al robot cuando sea necesario (puesto que en esta función se entra constantemente, así evitamos que se sobreescriba el destino todo el tiempo)
            self.new_init_pose = False  

            print("[#] Enter initial position for robot tb3_"+str(self.ID)+" in RViz")

            while self.PoseDefined() == False:
                None

            print("[#] Getting robot tb3_"+str(self.ID)+" to initial pos...")

            point = self.init_poses[self.ID]

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "tb3_" + str(self.ID) + "/map" # Reference
            goal.target_pose.pose.position.x = point[0]
            goal.target_pose.pose.position.y = point[1]
            goal.target_pose.pose.orientation.z = 0
            goal.target_pose.pose.orientation.w = 1

            self.client.send_goal(goal) # Send objective point

        if self.client.get_state() is GoalStatus.SUCCEEDED and self.ready == False:
            self.ready = True
            # self.new_init_pose = True
            print("[#] Robot tb3_"+str(self.ID)+" ready")
            # self.init_poses = []

    def stop(self):    
        pose = self.getCurrentPos()
        self.goToGoal(pose,False)
        self.new_goal=True
        # self.client.cancel_goal()

    def stop_exploring(self):
        nodes = os.popen("rosnode list").readlines()
        nodes = [node.replace("\n","") for node in nodes]
        if '/explore' in nodes:
            os.system("rosnode kill /explore")

    # Funciones para los estados, que se lanzan desde los threads de la máquina de estados
    def INITIALIZE(self):
        self.setHome()
        self.setInitialPose()

    def WAIT(self):
        self.stop()
        self.stop_exploring()

    def HOMING(self):
        self.stop_exploring()
        self.goToGoal(self.home,True)

    def EXPLORE(self):
        self.new_init_pose = True
        self.init_poses = []
        print("[#] Exploring environment...")
        os.system("ROS_NAMESPACE=tb3_"+str(self.ID) + " roslaunch explore_lite explore.launch costmap_topic:=/map costmap_updates_topic:=/map_updates robot_base_frame:=tb3_"+str(self.ID)+"/base_link > /dev/null")
        # os.system("ROS_NAMESPACE=tb3_"+str(self.ID) + " roslaunch explore_lite explore_costmap.launch robot_base_frame:=tb3_"+str(self.ID)+"/base_link > /dev/null")

