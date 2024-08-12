#!/usr/bin/env python3

import os
import random
import sys
import time
from time import sleep
from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose,Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_srvs.srv import Empty


class DQNGazeboOne(Node):
    def __init__(self):
        super().__init__('dqn_gazebo_1')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        # Entity 'goal'
        self.entity_dir_path = os.path.dirname(os.path.realpath(__file__))
        print(self.entity_dir_path)
        self.entity_dir_path = self.entity_dir_path.replace('lib/turtlebot','share/turtlebot/models/turtlebot3_dqn_world/goal_box')
        self.entity_path = os.path.join(self.entity_dir_path, 'model.sdf')
        self.entity = open(self.entity_path, 'r').read()
        self.entity_name = 'goalbox_1'

        self.goal_pose_x = 0.0
        self.goal_pose_y = 1.5

        self.init_state = False

        """************************************************************
        ** Initialise ROS publishers, subscribers and clients
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.vel_pub = self.create_publisher(Twist,'/bot_1/cmd_vel',qos)
        self.goal_pose_pub = self.create_publisher(Pose, '/bot_1/goal_pose', qos)

        # Initialise client
        self.delete_entity_client = self.create_client(DeleteEntity,'delete_entity')
        self.spawn_entity_client = self.create_client(SpawnEntity, 'spawn_entity')
        self.reset_simulation_client = self.create_client(Empty, 'reset_simulation')

        # Initialise servers
        self.task_succeed_server = self.create_service(Empty,'bot_1/task_succeed',self.task_succeed_callback)
        self.task_fail_server = self.create_service(Empty, 'bot_1/task_fail', self.task_fail_callback)

        # Process
        self.publish_timer = self.create_timer(0.01,self.publish_callback)

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def publish_callback(self):
        # Init
        if self.init_state is False:
            self.delete_entity()
            self.reset_simulation()
            self.init_state = True
            print("init!!!")
            print("Goal pose: ", self.goal_pose_x, self.goal_pose_y)

        # Publish goal pose
        goal_pose = Pose()
        goal_pose.position.x = self.goal_pose_x
        goal_pose.position.y = self.goal_pose_y
        self.goal_pose_pub.publish(goal_pose)
        self.spawn_entity()

    def task_succeed_callback(self, request, response):
        self.delete_entity()
        self.generate_goal_pose()
        print("generate a new goal :)")

        return response

    def task_fail_callback(self, request, response):
        self.delete_entity()
        #self.reset_simulation()
        self.move_back()
        self.generate_goal_pose()
        print("reset the gazebo environment :(")

        return response

    def move_back(self):
        t = time.time()
        vel = Twist()
        vel.linear.x = -1.0
        while t+2.0 > t:
            self.vel_pub.publish(vel)
            t = time.time()
        self.get_logger().info('Moved Backwards')
            
    def generate_goal_pose(self):
        goal_pose_list = [[1.0, 0.0], [2.0, -1.5], [0.0, -2.0], [2.0, 2.0], [0.8, 2.0],
                            [-1.9, 1.9], [-1.9, 0.2], [-1.9, -0.5], [-2.0, -2.0], [-0.5, -1.0]]
        index = random.randrange(0, 10)
        self.goal_pose_x = goal_pose_list[index][0]
        self.goal_pose_y = goal_pose_list[index][1]
        print("Goal pose: ", self.goal_pose_x, self.goal_pose_y)

    def reset_simulation(self):
        req = Empty.Request()
        while not self.reset_simulation_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Reset service not available, waiting again...')
        while not self.reset_simulation_client.service_is_ready():
            self.get_logger().info('Delete entity service not ready, waiting...')
            sleep(1)
        self.reset_simulation_client.call_async(req)

    def delete_entity(self):
        req = DeleteEntity.Request()
        req.name = self.entity_name
        while not self.delete_entity_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Delete service not available, waiting again...')
        while not self.delete_entity_client.service_is_ready():
            self.get_logger().info('Delete entity service not ready, waiting...')
            sleep(1)
        self.delete_entity_client.call_async(req)

    def spawn_entity(self):
        goal_pose = Pose()
        goal_pose.position.x = self.goal_pose_x
        goal_pose.position.y = self.goal_pose_y
        req = SpawnEntity.Request()
        req.name = self.entity_name
        req.xml = self.entity
        req.initial_pose = goal_pose
        while not self.spawn_entity_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Spawn service not available, waiting again...')
        while not self.spawn_entity_client.service_is_ready():
            self.get_logger().info('Delete entity service not ready, waiting...')
            sleep(1)
        self.spawn_entity_client.call_async(req)

#def main(args=sys.argv[1]):
def main(args=None):
    rclpy.init(args=args)
    dqn_gazebo_1 = DQNGazeboOne()
    rclpy.spin(dqn_gazebo_1)
    dqn_gazebo_1.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
