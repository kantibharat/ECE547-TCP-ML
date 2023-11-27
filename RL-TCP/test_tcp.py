#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
from ns3gym import ns3env
from tcp_base import TcpTimeBased
from tcp_newreno import TcpNewReno

#import libraries for RL network
import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
from tensorflow import keras

__author__ = "Piotr Gawlowicz"
__copyright__ = "Copyright (c) 2018, Technische Universität Berlin"
__version__ = "0.1.0"
__email__ = "gawlowicz@tkn.tu-berlin.de"


parser = argparse.ArgumentParser(description='Start simulation script on/off')
parser.add_argument('--start',
                    type=int,
                    default=1,
                    help='Start ns-3 simulation script 0/1, Default: 1')
parser.add_argument('--iterations',
                    type=int,
                    default=1,
                    help='Number of iterations, Default: 1')
args = parser.parse_args()
startSim = bool(args.start)
iterationNum = int(args.iterations)

port = 5555
simTime = 10 # seconds
stepTime = 0.5  # seconds
seed = 12
simArgs = {"--duration": simTime,}
debug = False

env = ns3env.Ns3Env(port=port, stepTime=stepTime, startSim=startSim, simSeed=seed, simArgs=simArgs, debug=debug)
# simpler:
#env = ns3env.Ns3Env()
env.reset()

ob_space = env.observation_space
ac_space = env.action_space

fp = open("output.txt", "w")

print("Observation space: ", ob_space,  ob_space.dtype)
print("Action space: ", ac_space, ac_space.dtype)
fp.write("Observation space: " + repr(ob_space) +  ", " + repr(ob_space.dtype) + "\n")
fp.write("Action space: " + repr(ac_space) + ", " + repr(ac_space.dtype) + "\n")

stepIdx = 0
currIt = 0

def get_agent(obs):
    socketUuid = obs[0]
    tcpEnvType = obs[1]
    tcpAgent = get_agent.tcpAgents.get(socketUuid, None)
    if tcpAgent is None:
        if tcpEnvType == 0:
            # event-based = 0
            tcpAgent = TcpNewReno()
            fp.write("Using New Reno\n\n")
        else:
            # time-based = 1
            tcpAgent = TcpTimeBased()
            fp.write("Using Time-Based TCP\n\n")
        tcpAgent.set_spaces(get_agent.ob_space, get_agent.ac_space)
        get_agent.tcpAgents[socketUuid] = tcpAgent

    return tcpAgent

def newReno():
    try:
        while True:
            print("Start iteration: ", currIt)
            fp.write("Start iteration: " + repr(currIt) + "\n")
            obs = env.reset()
            reward = 0
            done = False
            info = None
            print("Step: ", stepIdx)
            print("---obs: ", obs)
            fp.write("Step: " + repr(stepIdx) + "\n")
            fp.write("---obs: " + repr(obs)+ "\n")

            # get existing agent of create new TCP agent if needed
            tcpAgent = get_agent(obs)

            while True:
                stepIdx += 1
                action = tcpAgent.get_action(obs, reward, done, info)
                print("---action: ", action)
                fp.write("---action: " + repr(action)+ "\n")

                print("Step: ", stepIdx)
                obs, reward, done, info = env.step(action)
                print("---obs, reward, done, info: ", obs, reward, done, info)
                # fp.write("---obs, reward, done, info: " + repr(obs) + repr(reward) + repr(done) + repr(info)+ "\n")
                fp.write("---obs: " + repr(obs) + "\n")
                fp.write("reward: " + repr(reward) + "\n")
                fp.write("done: " + repr(done) + "\n")
                fp.write("info: " + repr(info)+ "\n")

                # get existing agent of create new TCP agent if needed
                tcpAgent = get_agent(obs)

                if done:
                    stepIdx = 0
                    if currIt + 1 < iterationNum:
                        env.reset()
                    break

            currIt += 1
            if currIt == iterationNum:
                break

    except KeyboardInterrupt:
        print("Ctrl-C -> Exit")
    finally:
        env.close()
        fp.write("Done")
        fp.close()
        print("Done")

# initialize variable
get_agent.tcpAgents = {}
get_agent.ob_space = ob_space
get_agent.ac_space = ac_space

state_size = ob_space.shape[0]
action_size = 3

fp.write("State size: " + repr(state_size) + "\n")
fp.write("Action size: " + repr(action_size) + "\n")

#define Keras model
model = keras.Sequential()
model.add(keras.layers.Dense(state_size, input_shape=(state_size,), activation='relu'))
model.add(keras.layers.Dense(state_size, input_shape=(state_size,), activation='relu'))
model.add(keras.layers.Dense(action_size, activation='softmax'))
model.compile(optimizer=tf.optimizers.Adam(learning_rate=0.001), loss='categorical_crossentropy', metrics=['accuracy'])

episodes = 1
max_steps = 100
env._max_episode_steps = max_steps

#exploration epsilon
# e = 1.0
e = 0.01
e_min = 0.01
e_decay = 0.999

#init
time_history = []
reward_history = []
cWnd_history=[]
cWnd_history2=[]
throughput=[]
delay=[]
RTT=[]
segACK=[]
tp_sum = 0
delay_sum = 0
num_step = 0
t2 = 10
t = []
action_mapping = {}
# action_mapping[0] = 0
# action_mapping[1] = 600
# action_mapping[2] = -60
action_mapping[0] = -1
action_mapping[1] = 0
action_mapping[2] = 1
action_mapping[3] = 3
U_new =0
U = 0
U_old = 0
reward = 0
new_ssThresh = 0

#from example
reward = 0
done = False
info = None

segmentsAckedSum = 0
bw = 1e7

for i in range(episodes):
    fp.write("Start episode: " + repr(i) + "\n")
    obs = env.reset()
    cWnd = obs[5]
    obs = np.reshape(obs, [1, state_size])
    sum_reward = 0

    for t in range(max_steps):
        if obs[0,4] == new_ssThresh:
            newReno()
            
        else:
            if np.random.rand(1) < e:
                action_ind = np.random.randint(3)
                fp.write("Action index: " + repr(action_ind) + "\n")
                fp.write("Value initialization ...\n")
            else:
                action_ind = np.argmax(model.predict(obs)[0])
                fp.write("Action index: " + repr(action_ind) + "\n")
        
        new_cWnd = cWnd + action_mapping[action_ind] // cWnd
        new_ssThresh = (cWnd / 2)    
        ## round to int
        new_ssThresh = new_ssThresh.astype('int32')

        action = [new_ssThresh, new_cWnd]
        # U_new = 0.7 * (np.log(obs[0,2])) - 0.7 * (np.log(obs[0,9]))
        d = obs[0,9] - obs[0,10] #RTT - RTT_min
        packet_loss = obs[0, 11]
        p = packet_loss / obs[0,6]
        
        #U = log(tp/B) - sigma1*log(d) + sigma2*log(1-p)
        if num_step > 0:
            U_new = np.log(tp) - np.log(bw) - 0.7 * (np.log(d)) + 0.7 * (np.log(1 - p)) #(np.log(obs[0,2])) - (np.log(obs[0,9]))
        else:
            U_new = 0.7 * (np.log(obs[0,2])) - 0.7 * (np.log(obs[0,9]))
        
        U = U_new - U_old
        
        # if U <= 0.05:
        #     reward = -5
        # elif U > 0.05:
        #     reward = 1
        # else:
        #     reward = 0

        if U >= 1:
            reward = 10
        elif U >= 0 and U < 1:
            reward = 2
        elif U >= -1 and U < 0:
            reward = -2
        else:
            reward = -10

        # next step
        next_obs, reward, done, info = env.step(action)
        cWnd = next_obs[5]
        fp.write("cWnd: " + repr(cWnd) + "\n")

        if done:
            fp.write("Episode: " + repr(i) + "/" + repr(episodes) + "\n")
            fp.write("Time: " + repr(t) + "\n")
            fp.write("Reward: " + repr(sum_reward) + "\n")
            fp.write("Epsilon: " + repr(e) + "\n")
            M_e = np.log(np.mean(throughput)) - 0.1*np.log(np.mean(delay))
            fp.write("M_e: " + repr(M_e) + "\n")
            break
        
        U_old = U_new
        next_obs = np.reshape(next_obs, [1, state_size])

        # Train
        target = reward
        if not done:
            target = (reward + 0.95 * np.amax(model.predict(next_obs)[0]))

        target_f = model.predict(obs)
        fp.write("Target: " + repr(target_f) + "\n")

        target_f[0][action_ind] = target
        model.fit(obs, target_f, epochs=1)

        obs = next_obs
        seg = obs[0,6]
        rtt = obs[0,9]
        sum_reward += reward
        if e > e_min:
            e *= e_decay
        num_step += 1

        segmentSize = obs[0,6]
        segmentsAckedSum += obs[0,7]
        tp = (segmentsAckedSum * segmentSize) / obs[0,2]
        tp_sum += tp

        rtt_min = obs[0,10]
        d = rtt - rtt_min
        delay_sum += d

        fp.write("Num steps: " + repr(num_step) + "\n")
        fp.write("Epsilon: " + repr(e) + "\n")
        fp.write("Reward (sum): " + repr(sum_reward) + "\n")
        fp.write("Throughput (sum): " + repr(tp_sum) + "\n")
        fp.write("Delay (sum): " + repr(delay_sum) + "\n")

        segACK.append(seg)
        RTT.append(rtt)
        cWnd_history.append(cWnd)
        time_history.append(t)
        reward_history.append(sum_reward)
        throughput.append(tp)
        delay.append(d)

    M_e = np.abs(np.log(np.mean(throughput)) - 0.1*np.log(np.mean(delay)))
    E_tp = np.mean(throughput)
    V_tp = np.var(throughput)
    E_d = np.mean(delay)
    V_d = np.var(delay)
    fp.write("M_e: " + repr(M_e) + "\n")
    fp.write("E(tp): " + repr(E_tp) + "\n")
    fp.write("V(tp): " + repr(E_tp) + "\n")
    fp.write("E(d): " + repr(E_d) + "\n")
    fp.write("V(d): " + repr(E_d) + "\n")

    # fp.write("Learning Performance\n")
    plt.rcdefaults()
    plt.rcParams.update({'font.size':16})
    fig, ax = plt.subplots(figsize=(10,4))
    plt.grid(True, linestyle='--')
    plt.title('Learning Performance')

    plt.plot(range(len(reward_history)), reward_history, label='Reward', marker="", linestyle="-", color='k')
    plt.plot(range(len(segACK)), segACK, label='segACK', marker="", linestyle="-", color='b')
    plt.plot(range(len(RTT)),RTT, label='Rtt', marker="", linestyle="-", color='y')
    
    plt.xlabel('Episode')
    plt.ylabel('Steps')
    plt.legend(prop={'size': 12})
    plt.savefig('learning.png', bbox_inches='tight')

    plt.rcdefaults()
    plt.rcParams.update({'font.size':16})
    fig, ax = plt.subplots(figsize=(10,4))
    plt.grid(True, linestyle='--')
    plt.title('Learning Performance')

    plt.plot(range(len(cWnd_history)), cWnd_history, label='cWnd', marker="", linestyle="-", color='g')

    plt.xlabel('Episode')
    plt.ylabel('cWnd')
    plt.legend(prop={'size': 12})
    plt.savefig('cWnd.png', bbox_inches='tight')

    plt.rcdefaults()
    plt.rcParams.update({'font.size':16})
    fig, ax = plt.subplots(figsize=(10,4))
    plt.grid(True, linestyle='--')
    plt.title('Throughput')
    plt.plot(range(len(throughput)), throughput, label='Throughput', marker="", linestyle="-", color='g')
    plt.xlabel('Episode')
    plt.ylabel('Steps')
    plt.legend(prop={'size': 12})
    plt.savefig('tp.png', bbox_inches='tight')

    plt.rcdefaults()
    plt.rcParams.update({'font.size':16})
    fig, ax = plt.subplots(figsize=(10,4))
    plt.grid(True, linestyle='--')
    plt.title('Delay')
    plt.plot(range(len(delay)), delay, label='Delay', marker="", linestyle="-", color='b')
    plt.xlabel('Episode')
    plt.ylabel('Steps')
    plt.legend(prop={'size': 12})
    plt.savefig('delay.png', bbox_inches='tight')