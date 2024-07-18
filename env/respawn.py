#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import time
import os
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose

class Respawn(Node):
    def __init__(self):
        super().__init__('Model_Spawner')
        self.modelPath = os.path.join("turtlebot","src","models","turtlebot3_square","goal_box","model.sdf")
        self.f = open(self.modelPath, 'r')
        self.model = self.f.read()
        self.stage = 4
        self.goal_position = Pose()
        self.init_goal_x = 0.6
        self.init_goal_y = 0.0
        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y
        self.modelName = 'goal'
        self.obstacle_1 = 0.6, 0.6
        self.obstacle_2 = 0.6, -0.6
        self.obstacle_3 = -0.6, 0.6
        self.obstacle_4 = -0.6, -0.6
        self.last_goal_x = self.init_goal_x
        self.last_goal_y = self.init_goal_y
        self.last_index = 0
        self.check_model = False
        self.index = 0
        
    def checkModel(self, model):
        self.check_model = False
        for i in range(len(model.name)):
            if model.name[i] == "goal":
                self.check_model = True

    def respawnModel(self):
        while True:
            if not self.check_model:
                spawn_client = self.create_client('spawn_entity',SpawnEntity)
                spawn_client.wait_for_service(timeout_sec=5.0)
                request = SpawnEntity.Request()
                request.name = self.modelName
                request.xml = self.model
                request.robot_namespace = 'bot_1'
                request.initial_pose = self.goal_position
                request.reference_frame = 'world'
                
                future = spawn_client.call_async(request)
                rclpy.spin_until_future_complete(self,future)
                if future.result() is not None:
                    self.get_logger.info("Goal position : %.1f, %.1f", self.goal_position.position.x, self.goal_position.position.y)
                    break
                else:
                    self.get_logger().error('Failed to spawn model')
                    if future.exception() is not None:
                        self.get_logger().error(future.exception())
            else:
                pass
            
    def deleteModel(self):
        while True:
            if self.check_model:
                del_client = self.create_client('delete_entity',DeleteEntity)
                del_client.wait_for_service(timeout_sec=5.0)
                
                request = DeleteEntity.Request()
                request.name = self.modelName
                
                future = del_client.call_async(request)
                rclpy.spin_until_future_complete(self,future)
                if future.result() is not None:
                    break
            else:
                pass

    def getPosition(self, position_check=False, delete=False):
        if delete:
            self.deleteModel()
       
        while position_check:
            goal_x_list = [0.6, 1.9, 0.5, 0.2, -0.8, -1, -1.9, 0.5, 2, 0.5, 0, -0.1, -2]
            goal_y_list = [0, -0.5, -1.9, 1.5, -0.9, 1, 1.1, -1.5, 1.5, 1.8, -1, 1.6, -0.8]

            self.index = random.randrange(0, 13)
            print(self.index, self.last_index)
            if self.last_index == self.index:
                    position_check = True
            else:
                self.last_index = self.index
                position_check = False

            self.goal_position.position.x = goal_x_list[self.index]
            self.goal_position.position.y = goal_y_list[self.index]

        time.sleep(0.5)
        self.respawnModel()

        self.last_goal_x = self.goal_position.position.x
        self.last_goal_y = self.goal_position.position.y

        return self.goal_position.position.x, self.goal_position.position.y