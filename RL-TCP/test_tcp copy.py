#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
from ns3gym import ns3env
from tcp_base import TcpTimeBased
from tcp_newreno import TcpNewReno

import numpy as np
import matplotlib.pyplot as plt

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

reward_history = []
cWnd_history=[]
throughput=[]
delay=[]
RTT=[]
segACK=[]
tp_sum = 0
delay_sum = 0
segmentsAckedSum = 0
sum_reward = 0

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

# initialize variable
get_agent.tcpAgents = {}
get_agent.ob_space = ob_space
get_agent.ac_space = ac_space

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

            cWnd = obs[5]

            seg = obs[6]
            rtt = obs[9]
            sum_reward += reward
            segmentSize = obs[6]
            segmentsAckedSum += obs[7]
            tp = (segmentsAckedSum * segmentSize) / obs[2]
            tp_sum += tp

            rtt_min = obs[10]
            d = rtt - rtt_min
            delay_sum += d

            segACK.append(seg)
            RTT.append(rtt)
            cWnd_history.append(cWnd)
            # time_history.append(t)
            reward_history.append(sum_reward)
            throughput.append(tp)
            delay.append(d)


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