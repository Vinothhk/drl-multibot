#!/usr/bin/env python3

import collections
import json
import numpy as np
import os
import random
import sys
import time
import psutil
from datetime import datetime
import rclpy
from rclpy.node import Node
from turtlebot3_msgs.srv import Dqn
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray

class QLearningAgent(Node):
    def __init__(self):
        super().__init__('q_learning_agent')

        qos = QoSProfile(depth=10)
        self.pub_result = self.create_publisher(Float32MultiArray, '/bot_1/result', qos)
        self.pub_action = self.create_publisher(Float32MultiArray, '/bot_1/action', qos)
        
        self.state_size = 4
        self.action_size = 5
        self.episode_size = 100

        # Q-learning hyperparameters
        self.discount_factor = 0.99
        self.learning_rate = 0.00025
        self.epsilon = 1.0
        self.epsilon_decay = 0.99
        self.epsilon_min = 0.05

        # Discretization parameters
        self.num_bins = [10, 10, 10, 10]  # Example: 10 bins for each dimension
        self.bin_edges = [np.linspace(0, 1, self.num_bins[i] + 1) for i in range(self.state_size)]

        # Q-table initialization
        self.q_table = np.zeros([len(edges) for edges in self.bin_edges] + [self.action_size])

        # Replay memory
        self.memory = collections.deque(maxlen=1000000)

        # Load saved models
        self.load_model = False
        self.load_episode = 0
        self.model_dir_path = os.path.dirname(os.path.realpath(__file__))
        self.model_dir_path = self.model_dir_path.replace("lib/turtlebot", "share/turtlebot/model_weights")
        self.model_path = os.path.join(self.model_dir_path, 'model_one' + '_episode' + str(self.load_episode) + '.npy')

        if self.load_model:
            self.q_table = np.load(self.model_path)
            with open(os.path.join(self.model_dir_path, 'model_one' + '_episode' + str(self.load_episode) + '.json')) as outfile:
                param = json.load(outfile)
                self.epsilon = param.get('epsilon')

        self.dqn_com_client = self.create_client(Dqn, 'bot_1/dqn_com')
        self.start_time = time.time()
        self.rewards = []
        self.successes = 0
        self.failures = 0
        self.episodes = 0
        self.evaluation_interval = 10

        self.exploration_actions = 0
        self.total_actions = 0

        self.coverage_area = set()
        self.total_area = 1000
        self.cpu_usages = []
        self.memory_usages = []

        self.process()

    def discretize_state(self, state):
        discretized_state = []
        for i in range(self.state_size):
            if i >= len(self.bin_edges):
                raise IndexError(f"Index {i} out of range for bin edges.")
            if state[i] < self.bin_edges[i][0] or state[i] > self.bin_edges[i][-1]:
                # Clamp state[i] to the range of bin edges if necessary
                state[i] = min(max(state[i], self.bin_edges[i][0]), self.bin_edges[i][-1])
            
            # Use np.digitize and adjust index
            bin_index = np.digitize(state[i], self.bin_edges[i]) - 1
            if bin_index < 0 or bin_index >= len(self.bin_edges[i]):
                print(f"Warning: Bin index out of range for state[{i}] = {state[i]}")
            
            discretized_state.append(bin_index)
        
        return tuple(discretized_state)

    def process(self):
        global_step = 0

        for episode in range(self.load_episode + 1, self.episode_size):
            self.get_logger().info(f'Ep No: {episode}\n')
            global_step += 1
            local_step = 0

            state = list()
            next_state = list()
            done = False
            init = True
            score = 0
            action_data = Float32MultiArray()
            result_data = Float32MultiArray()
            time.sleep(1.0)

            while not done:
                local_step += 1

                if local_step == 1:
                    action = 2  # Move forward
                else:
                    state = next_state
                    discretized_state = self.discretize_state(state)
                    action = int(self.get_action(discretized_state))
                    self.total_actions += 1

                req = Dqn.Request()
                req.action = action
                req.init = init
                while not self.dqn_com_client.wait_for_service(timeout_sec=3.0):
                    self.get_logger().info('Service not available, waiting again...')

                future = self.dqn_com_client.call_async(req)

                while rclpy.ok():
                    rclpy.spin_once(self)
                    if future.done():
                        if future.result() is not None:
                            next_state = future.result().state
                            reward = future.result().reward
                            done = future.result().done
                            score += reward
                            init = False
                            
                            position = (next_state[0], next_state[1])
                            self.coverage_area.add(position)

                            cpu_usage = psutil.cpu_percent()
                            memory_usage = psutil.virtual_memory().percent
                            self.cpu_usages.append(cpu_usage)
                            self.memory_usages.append(memory_usage)
                        else:
                            self.get_logger().error(
                                'Exception while calling service: {0}'.format(future.exception()))
                        break

                if local_step > 1:
                    discretized_next_state = self.discretize_state(next_state)
                    self.update_q_table(discretized_state, action, reward, discretized_next_state, done)

                    if done:
                        print(f'Score: {score}')
                        result_data.data = [float(score), float(np.max(self.q_table))]
                        self.pub_result.publish(result_data)
                        
                        print(
                            "Episode:", episode,
                            "Score:", score,
                            "Epsilon:", self.epsilon)

                        self.rewards.append(score)
                        self.episodes += 1

                        if reward == 5:
                            self.successes += 1
                        elif reward == -10:
                            self.failures += 1
                        
                        param_keys = ['epsilon']
                        param_values = [self.epsilon]
                        param_dictionary = dict(zip(param_keys, param_values))

                time.sleep(0.01)
            action_data.data = [float(action), score, reward]
            self.pub_action.publish(action_data)
            if episode % 10 == 0:
                np.save(self.model_path, self.q_table)
                with open(os.path.join(
                    self.model_dir_path,
                        'model_one'+'_episode'+str(episode)+'.json'), 'w') as outfile:
                    json.dump(param_dictionary, outfile)

            if self.epsilon > self.epsilon_min:
                self.epsilon *= self.epsilon_decay
            
            if episode == self.episode_size - 1:
                self.get_logger().info('Training Complete')
                self.print_evaluation_matrix()
            
            print(state)

    def get_action(self, state):
        if np.random.rand() <= self.epsilon:
            self.exploration_actions += 1
            return random.randrange(self.action_size)
        else:
            return np.argmax(self.q_table[state])

    def update_q_table(self, state, action, reward, next_state, done):
        q_value = self.q_table[state][action]
        if done:
            target = reward
        else:
            target = reward + self.discount_factor * np.max(self.q_table[next_state])
        self.q_table[state][action] += self.learning_rate * (target - q_value)

    def print_evaluation_matrix(self):
        end_time = time.time()
        elapsed_time = end_time - self.start_time
        average_reward = sum(self.rewards[-self.evaluation_interval:]) / self.evaluation_interval
        exploration_percentage = (self.exploration_actions / self.total_actions) * 100
        overlap_percentage = (len(self.coverage_area) / self.total_area) * 100
        average_cpu_usage = sum(self.cpu_usages[-self.evaluation_interval:]) / len(self.cpu_usages[-self.evaluation_interval:])
        average_memory_usage = sum(self.memory_usages[-self.evaluation_interval:]) / len(self.memory_usages[-self.evaluation_interval:])
        
        self.get_logger().info(f'Average Reward: {average_reward:.2f}')
        self.get_logger().info(f'Exploration Percentage: {exploration_percentage:.2f}%')
        self.get_logger().info(f'Overlap Percentage: {overlap_percentage:.2f}%')
        self.get_logger().info(f'Average CPU Usage: {average_cpu_usage:.2f}%')
        self.get_logger().info(f'Average Memory Usage: {average_memory_usage:.2f}%')
        self.get_logger().info(f"Time Taken {elapsed_time:.2f} seconds")

def main(args=None):
    rclpy.init(args=args)
    q_learning_agent = QLearningAgent()
    rclpy.spin(q_learning_agent)

    q_learning_agent.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
