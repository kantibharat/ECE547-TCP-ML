import pandas as pd

# Rerun network simulation
import subprocess
cmd = "(cd ns-allinone-3.40/ns-3.40 && ./ns3 run 'scratch/model.cc --num_flows=1 --duration=5000')"
returned_value = subprocess.call(cmd, shell=True)  # returns the exit code in unix
print('exit code:', returned_value)

column_names = ['time', 'nextTx', 'intersendTime', 'intersendTS', 'intersendEWMA', 'cwnd', 'ssth', 'rtt', 'rttTS',
                'nextRx', 'interarrivalTime', 'interarrivalTS', 'interarrivalEWMA', 'numSent', 'numLost', 'reward']
minRtt = 0.15
minInterarrival = 0
minIntersend = 0
data = pd.read_csv("ns-allinone-3.40/ns-3.40/data/model.data", sep=" ", names=column_names, header=2, on_bad_lines='warn')
# data.to_csv('ns-allinone-3.40/ns-3.40/data/model.csv') #view data
