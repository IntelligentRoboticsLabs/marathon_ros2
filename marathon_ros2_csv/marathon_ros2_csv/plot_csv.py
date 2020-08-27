from IPython.core.display import HTML, SVG
import pandas as pd
import numpy as np
import xport
import IPython
from ipywidgets import Layout
from ipywidgets import widgets
from IPython.display import display

import matplotlib.ticker as ticker
import matplotlib.cm as cm
import matplotlib as mpl
from matplotlib.gridspec import GridSpec

import matplotlib.pyplot as plt
import seaborn as sns

input = pd.read_csv('/home/jgines/Desktop/topics_to_csv_12_02_2020_17_21_44.csv')
source = []
#rango dinámico
for i in range(0,1857):
    for l in input:
        #if l == 'time':
        #    continue
        #if l == 'distance':
        #    continue
        #if l == 'recovery_behavior_executed':
        #    continue
        #if l == 'vel_x':
        #    continue
        #if l == 'vel_theta':
        #    continue

        new_reg = {
            'time':i,
            'distance':input['distance'][i],
            'recovery_behavior_executed':input['recovery_behavior_executed'][i],
            'vel_x':input['vel_x'][i],
            'vel_theta':input['vel_theta'][i]
            }
        source.append(new_reg)

data = pd.DataFrame(source)
#print(data)
sns.relplot(x="time", y="distance", kind="line", data=data)
plt.show()