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
import numpy as np
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

class DQNAgentTwo(Node):
    def __init__(self):
        super().__init__('dqn_agent_2')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        qos = QoSProfile(depth=10)
        self.pub_result = self.create_publisher(Float32MultiArray,'/bot_2/result',qos)
        self.pub_action = self.create_publisher(Float32MultiArray,'/bot_2/action',qos)
        
        # State size and action size
        self.state_size = 4
        self.action_size = 5
        self.episode_size = 100

        # DQN hyperparameter
        self.discount_factor = 0.99
        self.learning_rate = 0.00025
        self.epsilon = 1.0
        self.epsilon_decay = 0.99
        self.epsilon_min = 0.05
        self.batch_size = 64
        self.train_start = 64

        # Replay memory
        self.memory = collections.deque(maxlen=1000000)

        # Build model and target model
        self.model = self.build_model()
        self.target_model = self.build_model()
        self.update_target_model()
        self.update_target_model_start = 2000

        # Load saved models
        self.load_model = False
        self.load_episode = 0
        self.model_dir_path = os.path.dirname(os.path.realpath(__file__))
        print(self.model_dir_path)
        self.model_dir_path = self.model_dir_path.replace("lib/turtlebot","share/turtlebot/model_weights")
        self.model_path = os.path.join(self.model_dir_path,'model_two'+'_episode'+str(self.load_episode)+'.h5')

        if self.load_model:
            self.model.set_weights(load_model(self.model_path).get_weights())
            with open(os.path.join(self.model_dir_path,'model_two'+'_episode'+str(self.load_episode)+'.json')) as outfile:
                param = json.load(outfile)
                self.epsilon = param.get('epsilon')

        """************************************************************
        ** Initialise ROS clients
        ************************************************************"""
        # Initialise clients
        self.dqn_com_client = self.create_client(Dqn, 'bot_2/dqn_com')
        self.start_time = time.time()
        self.rewards = []
        self.successes = 0
        self.failures = 0
        self.episodes = 0
        self.evaluation_interval = 10  # Evaluate every 100 episodes
        self.q_val = 0.0
        # Exploration metrics
        self.exploration_actions = 0
        self.total_actions = 0

        # Overlap and system utilization metrics
        self.coverage_area = set()
        self.total_area = 1000  # Example total area; adjust according to your environment
        self.cpu_usages = []
        self.memory_usages = []
        """************************************************************
        ** Start process
        ************************************************************"""
        self.process()

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def process(self):
        global_step = 0

        for episode in range(self.load_episode+1, self.episode_size):
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
                req.action = action
                req.init = init
                while not self.dqn_com_client.wait_for_service(timeout_sec=3.0):
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

                # Save <s, a, r, s'> samples
                if local_step > 1:
                    self.append_sample(state, action, reward, next_state, done)

                    # Train model
                    if global_step > self.update_target_model_start:
                        self.train_model(True)
                    elif global_step > self.train_start:
                        self.train_model()
    
                    if done:
                        print(f'Reward Score: {score}')
                        #print(np.max(self.q_val))
                        result_data.data = [float(score), float(np.max(self.q_val))]
                        self.pub_result.publish(result_data)
                        # Update neural network
                        self.update_target_model()

                        print(
                            "Episode:", episode,
                            "score:", score,
                            "memory length:", len(self.memory),
                            "epsilon:", self.epsilon)

                        self.rewards.append(score)
                        self.episodes += 1

                        if reward == 5:
                            self.successes += 1
                        elif reward == -10:
                            self.failures += 1
                        
                        param_keys = ['epsilon']
                        param_values = [self.epsilon]
                        param_dictionary = dict(zip(param_keys, param_values))

                # While loop rate
                time.sleep(0.01)
            action_data.data = [float(action), score, reward]
            self.pub_action.publish(action_data)
            # Update result and save model every 10 episodes
            if episode % 10 == 0:
                self.model_path = os.path.join(
                    self.model_dir_path,
                    'model_two'+'_episode'+str(episode)+'.h5')
                self.model.save(self.model_path)
                with open(os.path.join(
                    self.model_dir_path,
                        'model_two'+'_episode'+str(episode)+'.json'), 'w') as outfile:
                    json.dump(param_dictionary, outfile)

            # Epsilon
            if self.epsilon > self.epsilon_min:
                self.epsilon *= self.epsilon_decay
            
            if episode == self.episode_size-1:
                self.get_logger().info('Training Complete')
                self.print_evaluation_matrix()
            
            #print(state)
                
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

    def update_target_model(self):
        self.target_model.set_weights(self.model.get_weights())

    def get_action(self, state):
        if numpy.random.rand() <= self.epsilon:
            self.exploration_actions += 1
            return random.randrange(self.action_size)
        else:
            state = numpy.asarray(state)
            q_value = self.model.predict(state.reshape(1, len(state)))
            self.q_val  = q_value
            #print(numpy.argmax(q_value[0]))
            return numpy.argmax(q_value[0])

    def append_sample(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def train_model(self, target_train_start=False):
        mini_batch = random.sample(self.memory, self.batch_size)
        x_batch = numpy.empty((0, self.state_size), dtype=numpy.float64)
        y_batch = numpy.empty((0, self.action_size), dtype=numpy.float64)

        for i in range(self.batch_size):
            state = numpy.asarray(mini_batch[i][0])
            action = numpy.asarray(mini_batch[i][1])
            reward = numpy.asarray(mini_batch[i][2])
            next_state = numpy.asarray(mini_batch[i][3])
            done = numpy.asarray(mini_batch[i][4])

            q_value = self.model.predict(state.reshape(1, len(state)))
            self.max_q_value = numpy.max(q_value)

            if not target_train_start:
                target_value = self.model.predict(next_state.reshape(1, len(next_state)))
            else:
                target_value = self.target_model.predict(next_state.reshape(1, len(next_state)))

            if done:
                next_q_value = reward
            else:
                next_q_value = reward + self.discount_factor * numpy.amax(target_value)

            x_batch = numpy.append(x_batch, numpy.array([state.copy()]), axis=0)

            y_sample = q_value.copy()
            y_sample[0][action] = next_q_value
            y_batch = numpy.append(y_batch, numpy.array([y_sample[0]]), axis=0)

            if done:
                x_batch = numpy.append(x_batch, numpy.array([next_state.copy()]), axis=0)
                y_batch = numpy.append(y_batch, numpy.array([[reward] * self.action_size]), axis=0)

        self.model.fit(x_batch, y_batch, batch_size=self.batch_size, epochs=1, verbose=0)

    def print_evaluation_matrix(self):
        end_time = time.time()
        elapsed_time = end_time - self.start_time
        average_reward = sum(self.rewards[-self.evaluation_interval:]) / self.evaluation_interval
        exploration_percentage = (self.exploration_actions / self.total_actions) * 100
        overlap_percentage = (len(self.coverage_area) / self.total_area) * 100
        average_cpu_usage = sum(self.cpu_usages[-self.evaluation_interval:]) / len(self.cpu_usages[-self.evaluation_interval:])
        average_memory_usage = sum(self.memory_usages[-self.evaluation_interval:]) / len(self.memory_usages[-self.evaluation_interval:])
        
        #self.get_logger().info(f'Average Reward: {average_reward:.2f}')
        self.get_logger().info(f'Exploration Percentage: {exploration_percentage:.2f}%')
        self.get_logger().info(f'Success rate: {self.successes/self.episode_size:.2f}')
        self.get_logger().info(f'Cumulative rewards: {sum(self.rewards)}')
        #self.get_logger().info(f'Overlap Percentage: {overlap_percentage:.2f}%')
        self.get_logger().info(f'Average CPU Usage: {average_cpu_usage:.2f}%')
        self.get_logger().info(f'Average Memory Usage: {average_memory_usage:.2f}%')
        self.get_logger().info(f"Time Taken {elapsed_time:.2f} seconds")
        
def main(args=None):
    rclpy.init(args=args)
    dqn_agent_2 = DQNAgentTwo()
    rclpy.spin(dqn_agent_2)

    dqn_agent_2.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()