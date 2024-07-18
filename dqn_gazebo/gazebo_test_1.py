#!/usr/bin/env python3

import os
import random
import sys
from time import sleep
from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import PoseStamped, Pose
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
        self.goal_pose_pub = self.create_subscription(PoseStamped, 'bot_1/goal_pose',self.generate_goal_pose, qos)

        # Initialise client
        self.delete_entity_client = self.create_client(DeleteEntity,'delete_entity')
        self.spawn_entity_client = self.create_client(SpawnEntity, 'spawn_entity')
        self.reset_simulation_client = self.create_client(Empty, 'reset_simulation')

        # Initialise servers
        self.task_succeed_server = self.create_service(Empty,'bot_1/task_succeed',self.task_succeed_callback)
        self.task_fail_server = self.create_service(Empty, 'bot_1/task_fail', self.task_fail_callback)

        # Process
        #self.publish_timer = self.create_timer(0.01,self.publish_callback)

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def task_succeed_callback(self, request, response):
        self.delete_entity()
        print("generate a new goal :)")

        return response

    def task_fail_callback(self, request, response):
        self.delete_entity()
        print("reset the gazebo environment :(")

        return response

    def generate_goal_pose(self,msg):
        self.goal_pose_x = msg.pose.position.x
        self.goal_pose_y = msg.pose.position.y
        print("Goal pose: ", self.goal_pose_x, self.goal_pose_y)
        self.spawn_entity()

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
