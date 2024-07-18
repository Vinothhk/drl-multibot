#!/usr/bin/env python3

import math
import numpy

from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf_transformations import euler_from_quaternion
from turtlebot3_msgs.srv import Dqn


class DQNEnvironmentOne(Node):
    def __init__(self):
        super().__init__('dqn_environment_1')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_theta = 0.0

        self.action_size = 5
        self.done = False
        self.fail = False
        self.succeed = False

        self.goal_angle = 0.0
        self.goal_distance = 1.0
        self.init_goal_distance = 1.0
        self.scan_ranges = []
        self.min_obstacle_distance = 10.0
        self.min_obstacle_angle = 10.0

        self.local_step = 0
        self.goal_status = False
        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/bot_1/cmd_vel', qos)

        # Initialise subscribers
        self.goal_pose_sub = self.create_subscription(PoseStamped,'bot_1/goal_pose',self.goal_pose_callback,10)
        self.odom_sub = self.create_subscription(Odometry,'bot_1/odom',self.odom_callback,10)
        self.scan_sub = self.create_subscription(LaserScan,'bot_1/scan',self.scan_callback,qos_profile=qos_profile_sensor_data)

        # Initialise client
        self.task_succeed_client = self.create_client(Empty, 'bot_1/task_succeed')
        self.task_fail_client = self.create_client(Empty, 'bot_1/task_fail')

        # Initialise servers
        self.dqn_com_server = self.create_service(Dqn, 'bot_1/dqn_com', self.dqn_com_callback)
        print('init')
    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def eulerfromquaternion(self, quat):
        (roll, pitch, yaw) = euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])
        return yaw
    
    def goal_pose_callback(self, msg):
        self.goal_pose_x = float(msg.pose.position.x)
        self.goal_pose_y = float(msg.pose.position.y)
        
    def odom_callback(self, msg):
        self.last_pose_x = float(msg.pose.pose.position.x)
        self.last_pose_y = float(msg.pose.pose.position.y)
        self.last_pose_theta = self.eulerfromquaternion(msg.pose.pose.orientation)

        goal_distance = math.sqrt((self.goal_pose_x-self.last_pose_x)**2+ (self.goal_pose_y-self.last_pose_y)**2)

        path_theta = math.atan2(
            self.goal_pose_y-self.last_pose_y,
            self.goal_pose_x-self.last_pose_x)

        goal_angle = path_theta - self.last_pose_theta
        if goal_angle > math.pi:
            goal_angle -= 2 * math.pi

        elif goal_angle < -math.pi:
            goal_angle += 2 * math.pi

        self.goal_distance = goal_distance
        self.goal_angle = goal_angle
        
    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.min_obstacle_distance = min(self.scan_ranges)
        self.min_obstacle_angle = (numpy.argmin(self.scan_ranges)/180)*math.pi
        
    def get_state(self):
        state = list()
        state.append(float(self.goal_distance))
        state.append(float(self.goal_angle))
        state.append(float(self.min_obstacle_distance))
        state.append(float(self.min_obstacle_angle))
        self.local_step += 1

        # Succeed
        if self.goal_distance < 0.20:  # unit: m
            print("Goal! :)")
            self.succeed = True
            self.done = True
            vel = Twist()
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.linear.z = 0.0
            vel.angular.x = 0.0
            vel.angular.y = 0.0
            vel.angular.z = 0.0
            self.cmd_vel_pub.publish(vel)  # robot stop
            self.local_step = 0
            req = Empty.Request()
            while not self.task_succeed_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.task_succeed_client.call_async(req)

        # Fail
        if self.min_obstacle_distance < 0.13:  # unit: m
            print("Collision! :(")
            self.fail = True
            self.done = True
            vel = Twist()
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.linear.z = 0.0
            vel.angular.x = 0.0
            vel.angular.y = 0.0
            vel.angular.z = 0.0
            self.cmd_vel_pub.publish(vel)  # r # robot stop
            self.local_step = 0
            req = Empty.Request()
            while not self.task_fail_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.task_fail_client.call_async(req)

        if self.local_step == 500:
            print("Time out! :(")
            self.done = True
            self.local_step = 0
            req = Empty.Request()
            while not self.task_fail_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.task_fail_client.call_async(req)

        return state
    
    def reset(self):
        return self.state

    def dqn_com_callback(self, request, response):
            action = request.action
            twist = Twist()
            twist.linear.x = 0.3
            twist.angular.z = ((self.action_size - 1)/2 - action) * 1.5
            self.cmd_vel_pub.publish(twist)

            response.state = self.get_state()
            response.reward = self.get_reward(action)
            response.done = self.done

            if self.done is True:
                self.done = False
                self.succeed = False
                self.fail = False

            if request.init is True:
                self.init_goal_distance = math.sqrt(
                    (self.goal_pose_x-self.last_pose_x)**2
                    + (self.goal_pose_y-self.last_pose_y)**2)
            return response
            
        
    def get_reward(self, action):
        yaw_reward = 1 - 2*math.sqrt(math.fabs(self.goal_angle / math.pi))

        distance_reward = (2 * self.init_goal_distance) / \
            (self.init_goal_distance + self.goal_distance) - 1

        # Reward for avoiding obstacles
        if self.min_obstacle_distance < 0.25:
            obstacle_reward = -2
        else:
            obstacle_reward = 0

        reward = yaw_reward + distance_reward + obstacle_reward

        # + for succeed, - for fail
        if self.succeed:
            reward += 5
        elif self.fail:
            reward -= -10

        return reward

    """*******************************************************************************
    ** Below should be replaced when porting for ROS 2 Python tf_conversions is done.
    *******************************************************************************"""


def main(args=None):
    rclpy.init(args=args)
    dqn_environment_1 = DQNEnvironmentOne()
    rclpy.spin(dqn_environment_1)

    dqn_environment_1.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
