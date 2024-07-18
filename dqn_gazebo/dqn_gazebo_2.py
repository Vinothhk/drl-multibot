#!/usr/bin/env python3
import os
import random
import sys

from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_srvs.srv import Empty
from time import sleep

class DQNGazeboTwo(Node):
    def __init__(self):
        super().__init__('dqn_gazebo_2')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        # Entity 'goal'
        self.entity_dir_path = os.path.dirname(os.path.realpath(__file__))
        print(self.entity_dir_path)
        self.entity_dir_path = self.entity_dir_path.replace('lib/turtlebot','share/turtlebot/models/turtlebot3_dqn_world/goal_box')
        self.entity_path = os.path.join(self.entity_dir_path, 'model.sdf')
        self.entity = open(self.entity_path, 'r').read()
        self.entity_name = 'goalbox_2'

        self.goal_pose_x = 0.5
        self.goal_pose_y = 0.0

        self.init_state = False

        """************************************************************
        ** Initialise ROS publishers, subscribers and clients
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.goal_pose_pub = self.create_publisher(Pose, '/bot_2/goal_pose', qos)

        # Initialise client
        self.delete_entity_client = self.create_client(DeleteEntity, 'delete_entity')
        self.spawn_entity_client = self.create_client(SpawnEntity, 'spawn_entity')
        self.reset_simulation_client = self.create_client(Empty, 'reset_simulation')

        # Initialise servers
        self.task_succeed_server = self.create_service(Empty,'bot_2/task_succeed',self.task_succeed_callback)
        self.task_fail_server = self.create_service(Empty, 'bot_2/task_fail', self.task_fail_callback)

        # Process
        self.publish_timer = self.create_timer(
            0.01,  # unit: s
            self.publish_callback)

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
        self.reset_simulation()
        self.generate_goal_pose()
        print("reset the gazebo environment :(")

        return response

    def generate_goal_pose(self):
        goal_pose_list = [[-3.1, -3.1], [3.2, 3.3], [4.4, -0.7], [8.4, 3.5], [8.5, 4.7],
                              [-8.0, -1.9], [-3.7, -3.7], [-1.57, 1.5], [0.0, -4.2], [1.8, 5.1]]
        index = random.randrange(0, 10)
        self.goal_pose_x = goal_pose_list[index][0]
        self.goal_pose_y = goal_pose_list[index][1]
        print("Goal pose: ", self.goal_pose_x, self.goal_pose_y)

    def reset_simulation(self):
        req = Empty.Request()
        while not self.reset_simulation_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.reset_simulation_client.service_is_ready():
            self.get_logger().info('Delete entity service not ready, waiting...')
            sleep(1)
        self.reset_simulation_client.call_async(req)
        
        """ future = self.reset_simulation_client.call_async(req)
        rclpy.spin_until_future_complete(self, future) """

    def delete_entity(self):
        req = DeleteEntity.Request()
        req.name = self.entity_name
        while not self.delete_entity_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.delete_entity_client.service_is_ready():
            self.get_logger().info('Delete entity service not ready, waiting...')
        self.delete_entity_client.call_async(req)
        
        """ future = self.delete_entity_client.call_async(req)
        rclpy.spin_until_future_complete(self, future) """

    def spawn_entity(self):
        goal_pose = Pose()
        goal_pose.position.x = self.goal_pose_x
        goal_pose.position.y = self.goal_pose_y
        req = SpawnEntity.Request()
        req.name = self.entity_name
        req.xml = self.entity
        req.initial_pose = goal_pose
        while not self.spawn_entity_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.spawn_entity_client.service_is_ready():
            self.get_logger().info('Delete entity service not ready, waiting...')
        self.spawn_entity_client.call_async(req)

#def main(args=sys.argv[1]):
def main(args=None):
    rclpy.init(args=args)
    dqn_gazebo_2 = DQNGazeboTwo()
    rclpy.spin(dqn_gazebo_2)
    dqn_gazebo_2.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
