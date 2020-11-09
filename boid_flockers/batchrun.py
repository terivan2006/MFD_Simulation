# -*- coding: utf-8 -*-
"""
Created on Tue Sep 15 09:38:46 2020

@author: teriv
"""
#%%
import os
os.chdir('C:\\Books\\simulation\\MFD_Simulation\\MFD_Simulation\\boid_flockers')

from boid import Boid
from model import BoidFlockers
from mesa.batchrunner import BatchRunner
import matplotlib.pyplot as plt
import pandas as pd
from mesa.datacollection import DataCollector
import time
import numpy as np


#%%

#%%

model = BoidFlockers(rate=200, vision=4, separation=1)
a = time.time()
for i in range(500):
    model.step()
    try:
        print(len(model.space._agent_points))
    except: pass
    print('step # ', i, ' took ', (time.time()-a)*1000, 'ms')
    a=time.time()
    
#%%


#%%
fixed_params = {"width": 100,
               "height": 100,
               "vision": 4,
               "separation":2,
               "size_factor": 2,
               'speed':0.1}

variable_params = {"rate": [2,4]}

batch_run = BatchRunner(BoidFlockers,
                        variable_params,
                        fixed_params,
                        iterations=1,
                        max_steps=200,
                        model_reporters={"Data Collector": lambda m: \
                                         m.datacollector},
                        )

#%%

#%%
batch_run.run_all()
#%%


#%%
run_data = batch_run.get_model_vars_dataframe()
br_step_data = pd.DataFrame()
for i in range(len(run_data["Data Collector"])):
    if isinstance(run_data["Data Collector"][i], DataCollector):
        i_run_data = run_data["Data Collector"][i].get_agent_vars_dataframe()
        # i_run_data['time'] = i_run_data.index
        i_run_data['time'] = [x[0] for x in i_run_data.index]
        i_run_data['sim'] = str(i)
        br_step_data = br_step_data.append(i_run_data, ignore_index=True)
#run_data.head()
# plt.scatter(run_data.N, run_data.Gini)
#%%
        


#%%
plt.scatter(br_step_data.loc[br_step_data['time']<=1200,'Occupancy'],\
            br_step_data.loc[br_step_data['time']<=1200,'Speed'])
#%%