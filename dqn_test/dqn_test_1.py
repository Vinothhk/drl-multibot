#!/usr/bin/env python3

import collections
from keras.layers import Activation
from keras.layers import Dense
from keras.layers import Dropout
from keras.models import Sequential
from keras.models import load_model
from keras.optimizers import RMSprop
import json
import numpy
import os
import random
import sys
import time
from math import sqrt
import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from turtlebot3_msgs.srv import Dqn
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

class DQNTest(Node):
    def __init__(self):
        super().__init__('dqn_test')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        # State size and action size
        self.state_size = 4
        self.action_size = 5
        self.start_time = time.time()
        # DQN hyperparameter
        self.discount_factor = 0.99
        self.learning_rate = 0.00025
        self.epsilon = 1.0
        self.epsilon_decay = 0.99
        self.epsilon_min = 0.05
        self.batch_size = 64
        self.train_start = 64
        
        self.rewards = []
        self.successes = 0
        self.failures = 0
        self.episodes = 11
        self.evaluation_interval = 10  # Evaluate every 100 episodes

        # Exploration metrics
        self.exploration_actions = 0
        self.total_actions = 0

        # Overlap and system utilization metrics
        self.coverage_area = set()
        self.total_area = 1000  # Example total area; adjust according to your environment
        self.cpu_usages = []
        self.memory_usages = []

        # Replay memory
        self.memory = collections.deque(maxlen=1000000)

        # Build model and target model
        self.model = self.build_model()
        self.target_model = self.build_model()

        self.current_position = (0.0, 0.0)
        self.last_position = (0.0, 0.0)
        self.position_history = []
        
        # Load saved models
        self.load_model = True
        self.load_episode = 570 #110
        self.model_dir_path = os.path.dirname(os.path.realpath(__file__))
        self.model_dir_path = self.model_dir_path.replace("lib/turtlebot","share/turtlebot/model_weights")
        self.model_path = os.path.join(
            self.model_dir_path,
            'model_one'+'_episode'+str(self.load_episode)+'.h5')

        if self.load_model:
            self.model.set_weights(load_model(self.model_path).get_weights())
            with open(os.path.join(
                    self.model_dir_path,
                    'model_one'+'_episode'+str(self.load_episode)+'.json')) as outfile:
                param = json.load(outfile)
                self.epsilon = param.get('epsilon')

        """************************************************************
        ** Initialise ROS clients
        ************************************************************"""
        # Initialise clients
        self.dqn_com_client = self.create_client(Dqn, 'bot_1/dqn_com')
        self.odom_subscription = self.create_subscription(Odometry,'bot_1/odom',self.odom_callback,QoSProfile(depth=10))
        """************************************************************
        ** Start process
        ************************************************************"""
        self.process()

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def odom_callback(self, msg):
        # Update the current position from the odometry data
        self.last_position = self.current_position
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        # Store position for the current episode
        self.position_history.append(self.current_position)
        
    def process(self):
        global_step = 0

        for episode in range(1,self.episodes):
            print(f'Episode = {episode}')
            global_step += 1
            local_step = 0

            state = list()
            next_state = list()
            done = False
            init = True
            score = 0
            self.position_history = []
            # Reset DQN environment
            time.sleep(1.0)

            while not done:
                local_step += 1

                # Aciton based on the current state
                if local_step == 1:
                    action = 2  # Move forward
                else:
                    state = next_state
                    action = int(self.get_action(state))
                    self.total_actions += 1
                    
                # Send action and receive next state and reward
                req = Dqn.Request()
                #print(int(action))
                req.action = action
                req.init = init
                while not self.dqn_com_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting again...')

                future = self.dqn_com_client.call_async(req)

                while rclpy.ok():
                    rclpy.spin_once(self)
                    if future.done():
                        if future.result() is not None:
                            # Next state and reward
                            next_state = future.result().state
                            reward = future.result().reward
                            done = future.result().done
                            score += reward
                            init = False
                            
                            position = (next_state[0], next_state[1])
                            self.coverage_area.add(position)

                            # Track system utilization
                            cpu_usage = psutil.cpu_percent()
                            memory_usage = psutil.virtual_memory().percent
                            self.cpu_usages.append(cpu_usage)
                            self.memory_usages.append(memory_usage)
                        else:
                            self.get_logger().error(
                                'Exception while calling service: {0}'.format(future.exception()))
                        break
                        
                self.rewards.append(score)
                #self.episodes += 1

                if reward == 5:
                    self.successes += 1
                elif reward == -10:
                    self.failures += 1
                # While loop rate
                time.sleep(0.01)
            print(f'Episode {episode} done!')
            path_lenth = self.calculate_path_length(self.position_history)
            print(f'Path length  = {path_lenth}\n Score = {score}')
        #self.print_evaluation_matrix()
        self.get_logger().info('Testing Completed..')
        self.get_logger().info(f'Success Rate: {self.successes/self.episodes}')    
    
    # def print_evaluation_matrix(self):
    #     end_time = time.time()
    #     elapsed_time = end_time - self.start_time
    #     average_reward = sum(self.rewards[-self.evaluation_interval:]) / self.evaluation_interval
    #     exploration_percentage = (self.exploration_actions / self.total_actions) * 100
    #     overlap_percentage = (len(self.coverage_area) / self.total_area) * 100
    #     average_cpu_usage = sum(self.cpu_usages[-self.evaluation_interval:]) / len(self.cpu_usages[-self.evaluation_interval:])
    #     average_memory_usage = sum(self.memory_usages[-self.evaluation_interval:]) / len(self.memory_usages[-self.evaluation_interval:])
        
    #     self.get_logger().info(f'Average Reward: {average_reward:.2f}')
    #     self.get_logger().info(f'Exploration Percentage: {exploration_percentage:.2f}%')
    #     self.get_logger().info(f'Overlap Percentage: {overlap_percentage:.2f}%')
    #     self.get_logger().info(f'Average CPU Usage: {average_cpu_usage:.2f}%')
    #     self.get_logger().info(f'Average Memory Usage: {average_memory_usage:.2f}%')
    #     self.get_logger().info(f"Time Taken {elapsed_time:.2f} seconds")

    def build_model(self):
        model = Sequential()
        model.add(Dense(
            64,
            input_shape=(self.state_size,),
            activation='relu',
            kernel_initializer='lecun_uniform'))
        model.add(Dense(64, activation='relu', kernel_initializer='lecun_uniform'))
        model.add(Dropout(0.2))
        model.add(Dense(self.action_size, kernel_initializer='lecun_uniform'))
        model.add(Activation('linear'))
        model.compile(loss='mse', optimizer=RMSprop(lr=self.learning_rate, rho=0.9, epsilon=1e-06))
        model.summary()

        return model

    def get_action(self, state):
        if numpy.random.rand() <= self.epsilon:
            self.exploration_actions += 1
            return random.randrange(self.action_size)
        else:
            state = numpy.asarray(state)
            q_value = self.model.predict(state.reshape(1, len(state)))
            return numpy.argmax(q_value[0])
        
    def calculate_path_length(self, path):
        length = 0.0
        for i in range(1, len(path)):
            x1, y1 = path[i - 1]
            x2, y2 = path[i]
            length += sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return length
    
def main(args=None):
    rclpy.init(args=args)
    dqn_test = DQNTest()
    rclpy.spin(dqn_test)

    dqn_test.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
