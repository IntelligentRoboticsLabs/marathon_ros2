#!/usr/bin/env python
# coding: utf-8

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
exp_folder = "exp_3/teb_star/"
url = "/home/ubuntu/exp_data/" + exp_folder
n = len(os.listdir(url)) + 1
numberofcsvs = range(1,n)
extension = ".csv"
df=pd.DataFrame(columns=["time","distance","recoveries_executed","vel_x","vel_theta", "scan_min"])
for i in numberofcsvs:
    url_num=url+str(i)+extension
    df_ = pd.read_csv(url_num, keep_default_na= False)
    df = pd.concat([df, df_])

df.head(100)

i = 0
for n in df['time'].values:
  df['time'].values[i] = i
  i = i + 1


#plt.plot(df['distance'].values) #distancia por instante
#plt.xlabel('Time (s)')
#plt.ylabel('Distance (miles)')
#plt.title('Distance traveled')
#plt.savefig("figures/distance.pdf")
#plt.clf()

#plt.plot(df['distance'].values.cumsum()) # distancia acumulada
#plt.xlabel('Time (s)')
#plt.ylabel('Distance (miles)')
#plt.title('Distance traveled')
#plt.savefig("figures/distance_cumsum.pdf")
#plt.clf()



#plt.plot(df['distance'].values.cumsum(), linewidth=2) # distancia acumulada
#plt.xlabel('Time (s)')
#plt.ylabel('Distance (miles)')
#plt.title('Distance traveled')
#plt.savefig("figures/distance_cumsum.pdf")
#
#total_dist = df['distance'].values.cumsum()
#
#test = np.empty(len(total_dist), dtype=object)
#
#i = 0
#for n in df['recovery_behavior_executed'].values:
#  if n == '1':
#    test[i] = total_dist[i]
#  i = i + 1
#
#plt.plot(test, color='black', marker='|', markersize=7) 
#plt.xlabel('Time (s)')
#plt.ylabel('Distance (miles)')
#plt.title('Recovery behavior executed on time')
#plt.savefig("figures/behaviors_vs_distance.pdf")
#plt.clf()

#i = 0
#for n in df['psi_personal'].values:
#    df['psi_personal'].values[i] = n * 100.0
#    i = i + 1

print ("-------------- Extracted data from " + exp_folder + " ------------")
print("Total time (s): " + str((df['time'].values[-1] - df['time'].values[0])))
print("Total distance (m): " + str(df['distance'].values.cumsum()[-1]))
print("Linear vel: " + str(df['vel_x'].mean()) + " (" + str(df['vel_x'].std()) + ")")
print("Linear vel max: " + str(df['vel_x'].max()))
print("Min distance to an obstacle: " + str(df['scan_min'].min()))