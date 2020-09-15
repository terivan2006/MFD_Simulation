"""
Flockers
=============================================================
A Mesa implementation of Craig Reynolds's Boids flocker model.
Uses numpy arrays to represent vectors.
"""
import numpy as np
from mesa import Model
from mesa.space import ContinuousSpace
from mesa.time import RandomActivation
from mesa.datacollection import DataCollector
from .boid import Boid

def compute_N(model):
    try:
        N = len([agent for agent in model.schedule.agents \
                    if agent.pos[0] <= model.space.x_max/2 + \
                    model.space.x_max/2/model.size_factor and 
                    agent.pos[0] >= model.space.x_max/2 - \
                    model.space.x_max/2/model.size_factor and
                    agent.pos[1] <= model.space.y_max/2 + \
                    model.space.y_max/2/model.size_factor and 
                    agent.pos[1] >= model.space.y_max/2 - \
                    model.space.y_max/2/model.size_factor 
                    ])
        return N
    except: print('N error')
    
def compute_eff_flow(model):
    eff_flow = sum([agent.effective_speed for agent in model.schedule.agents \
                    if agent.pos[0] <= model.space.x_max/2 + \
                    model.space.x_max/2/model.size_factor and 
                    agent.pos[0] >= model.space.x_max/2 - \
                    model.space.x_max/2/model.size_factor and
                    agent.pos[1] <= model.space.y_max/2 + \
                    model.space.y_max/2/model.size_factor and 
                    agent.pos[1] >= model.space.y_max/2 - \
                    model.space.y_max/2/model.size_factor 
                    
                    ])
    return eff_flow
def compute_flow(model):
    flow = sum([agent.physic_speed for agent in model.schedule.agents \
                    if agent.pos[0] <= model.space.x_max/2 + \
                    model.space.x_max/2/model.size_factor and 
                    agent.pos[0] >= model.space.x_max/2 - \
                    model.space.x_max/2/model.size_factor and
                    agent.pos[1] <= model.space.y_max/2 + \
                    model.space.y_max/2/model.size_factor and 
                    agent.pos[1] >= model.space.y_max/2 - \
                    model.space.y_max/2/model.size_factor ])
    return flow
def compute_inp(model):
    inp = model.input_rate
    return inp*60

def compute_out(model):
    out = len(model.kill_agents)
    return out*60

def compute_speed(model):
    try:
        speed = sum([agent.physic_speed for agent in model.schedule.agents \
                    if agent.pos[0] <= model.space.x_max/2 + \
                    model.space.x_max/2/model.size_factor and 
                    agent.pos[0] >= model.space.x_max/2 - \
                    model.space.x_max/2/model.size_factor and
                    agent.pos[1] <= model.space.y_max/2 + \
                    model.space.y_max/2/model.size_factor and 
                    agent.pos[1] >= model.space.y_max/2 - \
                    model.space.y_max/2/model.size_factor ])/model.num_agents
        return speed
    except: print('speed error!')

def compute_eff_speed(model):
    try:
        eff_speed = sum([agent.effective_speed for agent in model.schedule.agents\
                    if agent.pos[0] <= model.space.x_max/2 + \
                    model.space.x_max/2/model.size_factor and 
                    agent.pos[0] >= model.space.x_max/2 - \
                    model.space.x_max/2/model.size_factor and
                    agent.pos[1] <= model.space.y_max/2 + \
                    model.space.y_max/2/model.size_factor and 
                    agent.pos[1] >= model.space.y_max/2 - \
                    model.space.y_max/2/model.size_factor ])/model.num_agents
        return eff_speed
    except: print('effective speed error')

class BoidFlockers(Model):
    """
    Flocker model class. Handles agent creation, placement and scheduling.
    """    
    def __init__(
        self,
        population=100,
        width=100,
        height=100,
        speed=1,
        vision=10,
        separation=2,
        rate = 10,
        size_factor = 2
        
    ):
        """
        Create a new Flockers model.

        Args:
            width, height: Size of the space.
            speed: How fast should the Boids move.
            vision: How far around should each Boid look for its neighbors
            separation: What's the minimum distance each Boid will attempt to
                    keep from any other
                    """
                    
        self.population = population
        self.unique_id = 1
        self.vision = vision
        self.speed = speed
        self.separation = separation
        self.schedule = RandomActivation(self)
        self.space = ContinuousSpace(width, height, False)
        self.size_factor = size_factor
        self.running = True
        self.rate = rate
        self.kill_agents = []
        self.input_rate = 0
        self.num_agents = 0
        self.running = True
        
        self.datacollector = DataCollector(
            model_reporters= {"Occupancy": compute_N,
                              "Total flow": compute_flow,
                              "Effective flow": compute_eff_flow,
                              "Inpute rate": compute_inp,
                              "Output rate": compute_out,
                              'Effective speed': compute_eff_speed ,
                              'Speed': compute_speed,
                              'size':'size_factor',
                              'inp_rate':'rate',
                              'vision':'vision',
                              'def_speed':'speed',
                              'sep':'separation'
                             },
            
            agent_reporters = {'id':'unique_id',
                               'prev_dist':'previos_distance',
                               'cur_dist':'current_distance',
                               'phys_speed':'physic_speed',
                               'vision':'vision',
                               'def_speed':'speed',
                               'sep':'separation'})
        
        
    def make_agents(self):
            x = self.space.x_max/2 + np.random.uniform(-1,1) * self.space.x_max/2/self.size_factor
            y = self.space.y_max/2 + np.random.uniform(-1,1) * self.space.y_max/2/self.size_factor
            pos = np.array((x, y))
            
            x_dest = self.space.x_max/2 + np.random.uniform(-1,1) * self.space.x_max/2/self.size_factor
            y_dest = self.space.y_max/2 + np.random.uniform(-1,1) * self.space.y_max/2/self.size_factor
            dest = np.array((x_dest, y_dest))
            
            velocity = np.random.random(2) * 2 - 1
            
            boid = Boid(
                unique_id=self.unique_id,
                model=self,
                pos=pos,
                speed = self.speed,
                velocity=velocity,
                destination = dest,
                vision=self.vision,
                separation=self.separation,
            )
            self.space.place_agent(boid, pos)
            self.schedule.add(boid)
        
    def step(self):
        """
        Create agents here
        """
        
        #collect data
        try:
            try:
                self.datacollector.collect(self)

            except: pass
            #compute input rate for step
            per_step = self.rate/60
            fractional = per_step % 1
            integer = int(per_step - round(fractional))
            num_frac = np.random.binomial(size=1, n=1, p= fractional)
            self.input_rate = int(integer+num_frac)
            #create agents
            for i in range(self.input_rate):
                self.make_agents()
                self.unique_id += 1
                self.num_agents += 1
            self.kill_agents = []
            #make 1 step
            self.schedule.step()
            #remove agents that arrived at their destinations
            for i in self.kill_agents:
                self.schedule.remove(i)
                self.num_agents -= 1
                self.space.remove_agent(i)
                
        except: print('cant make steps!')
