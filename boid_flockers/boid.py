import numpy as np

from mesa import Agent


class Boid(Agent):
    """
    A Boid-style flocker agent.

    The agent follows three behaviors to flock:
        - Cohesion: steering towards neighboring agents.
        - Separation: avoiding getting too close to any other agent.
        - Alignment: try to fly in the same direction as the neighbors.

    Boids have a vision that defines the radius in which they look for their
    neighbors to flock with. Their speed (a scalar) and velocity (a vector)
    define their movement. Separation is their desired minimum distance from
    any other Boid.
    """

    def __init__(
        self,
        unique_id,
        model,
        pos,
        speed,
        velocity,
        vision,
        separation,
        destination = np.zeros(2),
        init_time = 0,
        entry_time = 0
    ):
        """
        Create a new Boid flocker agent.

        Args:
            unique_id: Unique agent identifyer.
            pos: Starting position
            speed: Distance to move per step.
            heading: numpy vector for the Boid's direction of movement.
            vision: Radius to look around for nearby Boids.
            separation: Minimum distance to maintain from other Boids.
            cohere: the relative importance of matching neighbors' positions
            separate: the relative importance of avoiding close neighbors
            match: the relative importance of matching neighbors' headings

        """
        
        super().__init__(unique_id, model)
        self.pos = np.array(pos)
        self.speed = speed
        self.velocity = velocity
        self.vision = vision
        self.separation = separation
        self.od_dist = 0
        self.previos_distance = 0
        self.current_distance = 0
        self.destination = destination
        self.effective_speed = speed
        self.physic_speed = speed
        
        self.init_time = init_time
        self.entry_time = entry_time
        self.current_time = self.model.schedule.time 
        self.freeflow_endtime = 0
        
        self.n_conf = 0
        self.n_intrusion = 0
        
    def direct(self):
        towards_dest = self.model.space.get_heading(self.pos, self.destination)
        return towards_dest
    
    
    # def mvp(self, neighbor):
    #     #compute relative position w.r.t intruder
    #     x,y = self.pos-neighbor.pos
    #     dist_intruder = np.sqrt(x**2 + y**2)
    #     if dist_intruder < self.separation/2:
    #         self.model.n_intrusion += 1
    #     else: pass
    #     v_x,v_y = self.velocity - neighbor.velocity
    #     m = -v_y/(v_x) 
    #     b = y + x*v_y/(v_x)
    #     c_x = -m*b/(m**2 + 1)
    #     c_y = b/(m**2 + 1)
    #     #min distance from intruder
    #     dist_closest = np.sqrt(c_x**2 + c_y**2)
        
    #     if dist_closest < self.separation:
    #         self.model.n_confs += 1
    #         time_closest = (dist_closest)/self.speed
    #         factor = self.separation/(dist_closest) 
    #         co = np.array([c_x*factor,c_y*factor])
    #         delta_velocity = co/(time_closest)
    #     else:
    #         delta_velocity = np.zeros(2)
    #     return(delta_velocity)
        
    def mvp(self, neighbors):
        #compute relative position w.r.t intruder
        try:
            npos = np.array([x.pos for x in neighbors])
            nvel  = np.array([x.velocity for x in neighbors])
            x,y = (self.pos-npos)[:,0],(self.pos-npos)[:,1]
            
            dist_intruder = np.sqrt(x**2 + y**2)
            self.model.n_intrusion += int(sum((dist_intruder <= self.separation/2)*1))
            v_x,v_y = (self.velocity - nvel)[:,0],(self.velocity - nvel)[:,1]
            m = -v_y/(v_x) 
            b = y + x*v_y/(v_x)
            c_x = -m*b/(m**2 + 1)
            c_y = b/(m**2 + 1)
            #min distance from intruder
            dist_closest = np.sqrt(c_x**2 + c_y**2)
            
            sep = (dist_closest <= self.separation)*1
            self.model.n_confs += int(sum(sep))
            time_closest = (dist_closest)/self.speed
            factor = self.separation/(dist_closest) 
            co = np.array([c_x*factor,
                            c_y*factor])
            delta_velocity = co/(time_closest)

            return(delta_velocity.sum(axis=1))
        except: return np.array((0,0))
    
    def distance(self):
        x1, y1 = self.pos
        x2, y2 = self.destination
        dx = np.abs(x1 - x2)
        dy = np.abs(y1 - y2)
        return np.sqrt(dx * dx + dy * dy)  
    
         
    def step(self):
        """
        Get the Boid's neighbors, compute the new vector, and move accordingly.
        """
        try:
            self.n_conf = 0
            self.n_intrusion = 0
            self.current_time = self.model.schedule.time
            self.previos_distance = self.distance()
            
            neighbors = self.model.space.get_neighbors(self.pos, self.vision, False)

            self.velocity =  self.direct()/np.linalg.norm(self.direct())*self.speed
            
            self.velocity += self.mvp(neighbors)
            # for neighbor in neighbors:
                # self.velocity += 0#self.mvp(neighbor)
            
            self.velocity /= np.linalg.norm(self.velocity)
            
            # if np.linalg.norm(self.velocity) > self.speed:
            #     self.velocity /= np.linalg.norm(self.velocity)
            # else: pass

            new_pos = self.pos + self.velocity*self.speed
            
            self.physic_speed = np.linalg.norm(self.velocity)*self.speed
            self.model.space.move_agent(self, new_pos)
            
            self.current_distance = self.distance()
            self.effective_speed = self.previos_distance - self.current_distance
            
            self.freeflow_endtime = self.entry_time + self.od_dist/self.speed
            self.enroute_del = max(self.current_time - self.freeflow_endtime, 0)
            

            if self.distance() <= self.speed:
                self.model.kill_agents.append(self)
        except: print(self.unique_id, 'something went wrong!')