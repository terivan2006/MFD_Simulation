# -*- coding: utf-8 -*-
"""
Created on Tue Sep 15 09:38:46 2020

@author: teriv
"""
#%%
from boid import Boid
from model import BoidFlockers
from mesa.batchrunner import BatchRunner
import matplotlib.pyplot as plt
import pandas as pd
from mesa.datacollection import DataCollector

#%%

#%%
fixed_params = {"width": 100,
               "height": 100,
               "vision": 4,
               "separation":2,
               "size_factor": 2,
               'speed':0.1}

variable_params = {"rate": [20,40]}

batch_run = BatchRunner(BoidFlockers,
                        variable_params,
                        fixed_params,
                        iterations=1,
                        max_steps=5000,
                        model_reporters={"Data Collector": lambda m: m.datacollector})

#%%

#%%
batch_run.run_all()
#%%


#%%
run_data = batch_run.get_model_vars_dataframe()
br_step_data = pd.DataFrame()
for i in range(len(run_data["Data Collector"])):
    if isinstance(run_data["Data Collector"][i], DataCollector):
        i_run_data = run_data["Data Collector"][i].get_model_vars_dataframe()
        i_run_data['time'] = i_run_data.index
        i_run_data['sim'] = str(i)
        br_step_data = br_step_data.append(i_run_data, ignore_index=True)
#run_data.head()
# plt.scatter(run_data.N, run_data.Gini)
#%%
        
#%%
run_data = batch_run.get_agent_vars_dataframe()
br_step_data = pd.DataFrame()
for i in range(len(run_data["Data Collector"])):
    if isinstance(run_data["Data Collector"][i], DataCollector):
        i_run_data = run_data["Data Collector"][i].get_agent_vars_dataframe()
        i_run_data['time'] = i_run_data.index
        i_run_data['sim'] = str(i)
        br_step_data = br_step_data.append(i_run_data, ignore_index=True)
#run_data.head()
# plt.scatter(run_data.N, run_data.Gini)
#%%

#%%
plt.scatter(br_step_data.loc[br_step_data['time']<=1200,'Occupancy'],\
            br_step_data.loc[br_step_data['time']<=1200,'Speed'])
#%%