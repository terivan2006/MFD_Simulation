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
        cohere=0.025,
        separate=0.25,
        match=0.04,
        direct_factor = 0.25,
        destination = np.zeros(2)
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
        
        self.cohere_factor = cohere
        self.separate_factor = separate
        self.match_factor = match
        self.direct_factor = direct_factor
        
        self.previos_distance = 0
        self.current_distance = 0
        self.destination = destination
        self.effective_speed = speed
        self.physic_speed = speed
        
        
    def direct(self):
        towards_dest = self.model.space.get_heading(self.pos, self.destination)
        return towards_dest
    
    def cohere(self, neighbors):
        """
        Return the vector toward the center of mass of the local neighbors.
        """
        cohere = np.zeros(2)
        if neighbors:
            for neighbor in neighbors:
                cohere += self.model.space.get_heading(self.pos, neighbor.pos)
            cohere /= len(neighbors)
        return cohere
    
    def match_heading(self, neighbors):
        """
        Return a vector of the neighbors' average heading.
        """
        match_vector = np.zeros(2)
        if neighbors:
            for neighbor in neighbors:
                match_vector += neighbor.velocity
            match_vector /= len(neighbors)
        return match_vector
    
    def mvp(self, neighbor):
        #compute relative position w.r.t intruder
        x,y = self.pos-neighbor.pos
       # dist_intruder = np.sqrt(x**2 + y**2)
        v_x,v_y = self.velocity - neighbor.velocity
        m = -v_y/(v_x) 
        b = y + x*v_y/(v_x)
        c_x = -m*b/(m**2 + 1)
        c_y = b/(m**2 + 1)
        #min distance from intruder
        dist_closest = np.sqrt(c_x**2 + c_y**2)
        
        if dist_closest < self.separation:
            time_closest = (dist_closest)/self.speed
            factor = self.separation/(dist_closest) 
            co = np.array([c_x*factor,c_y*factor])
            delta_velocity = co/(time_closest)
        else:
            delta_velocity = np.zeros(2)
        return(delta_velocity)
        
    def separate(self, neighbors):
        """
        Return a vector away from any neighbors closer than separation dist.
        """
        me = self.pos
        them = (n.pos for n in neighbors)
        separation_vector = np.zeros(2)
        for other in them:
            if self.model.space.get_distance(me, other) < self.separation:
                separation_vector -= self.model.space.get_heading(me, other)
        return separation_vector

    def distance(self):
        x1, y1 = self.pos
        x2, y2 = self.destination
        dx = np.abs(x1 - x2)
        dy = np.abs(y1 - y2)
        # if self.torus:
        #     print('Haha')
        #     dx = min(dx, self.width - dx)
            # dy = min(dy, self.height - dy)
        return np.sqrt(dx * dx + dy * dy)  
    
         
    def step(self):
        """
        Get the Boid's neighbors, compute the new vector, and move accordingly.
        """
        try:
            self.previos_distance = self.distance()
            neighbors = self.model.space.get_neighbors(self.pos, self.vision, False)
            self.velocity =  self.direct()/np.linalg.norm(self.direct())*self.speed
            # print(self.unique_id,'initial', self.velocity)
            for neighbor in neighbors:
                self.velocity += self.mvp(neighbor)
            # print(self.unique_id,'post', self.velocity)
            if np.linalg.norm(self.velocity) > self.speed:
                self.velocity /= np.linalg.norm(self.velocity)
            else: pass

            new_pos = self.pos + self.velocity #* self.speed
            self.physic_speed = np.linalg.norm(self.velocity)
            self.model.space.move_agent(self, new_pos)
            
            self.current_distance = self.distance()
            self.effective_speed = self.previos_distance - self.current_distance
            
            if self.distance() <= self.speed:
                self.model.kill_agents.append(self)
            
        except: print(self.unique, 'something went wrong!')