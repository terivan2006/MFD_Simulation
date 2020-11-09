from mesa.visualization.ModularVisualization import ModularServer

from .model import BoidFlockers
from .SimpleContinuousModule import SimpleCanvas
from mesa.visualization.UserParam import UserSettableParameter
from mesa.visualization.modules import ChartModule


def boid_draw(agent):
    try:
        portrayal = {"Shape": "circle",
                     "Filled": "true",
                     "r": 2}
    
        if agent.effective_speed < 0:
            portrayal["Color"] = "red"
            portrayal["Layer"] = 0
        if (agent.effective_speed > 0) and (agent.effective_speed <= agent.speed*0.2):
            portrayal["Color"] = "orange"
            portrayal["Layer"] = 1
            portrayal["r"] = 2
        if (agent.effective_speed > agent.speed*0.2) and (agent.effective_speed <= agent.speed*0.4):
            portrayal["Color"] = "gold"
            portrayal["Layer"] = 1
            portrayal["r"] = 2
        if (agent.effective_speed > agent.speed*0.4) and (agent.effective_speed <= agent.speed*0.6):
            portrayal["Color"] = "yellow"
            portrayal["Layer"] = 1
            portrayal["r"] = 2
        if (agent.effective_speed > agent.speed*0.6) and (agent.effective_speed <= agent.speed*0.8):
            portrayal["Color"] = "yellowgreen"
            portrayal["Layer"] = 1
            portrayal["r"] = 2
        if (agent.effective_speed > agent.speed*0.8) and (agent.effective_speed <= agent.speed):
            portrayal["Color"] = "darkgreen"
            portrayal["Layer"] = 1
            portrayal["r"] = 2
    except: print('error!')
    return portrayal
    # return {"Shape": "circle", "r": 2, "Filled": "true", "Color": "Red"}

boid_canvas = SimpleCanvas(boid_draw, 500, 500)

model_params = {
    # "population": UserSettableParameter(
    #     "slider", "Population", 100, 10, 1000
    # ),
    "width": 100,
    "height": 100,
    "speed": UserSettableParameter(
        "number", "Speed", 1, 0, 10, 0.1
    ),
    "vision": UserSettableParameter(
        "slider", "Vision", 10, 0, 100, 1
    ),
    "separation": UserSettableParameter(
        "slider", "Separation", 2, 0, 10,0.1
    ),
    "rate": UserSettableParameter(
        "number", "Input rate", 10, 0, 1000, 0.1
    ),
    "size_factor": UserSettableParameter(
        "slider", "Size factor", 2, 1, 10, 0.1
    ),
    "angle_min": UserSettableParameter(
        "slider", "Direction minimum", -180, -180, 180, 0.1
    ),
    "angle_max": UserSettableParameter(
        "slider", "Direction maximum", 180, -180, 180, 0.1
    )
}

chart_1 = ChartModule([{"Label": "Occupancy",
                      "Color": "Red"},
                        {"Label": "Queue length",
                       "Color": "Blue"}],
                    data_collector_name='datacollector')
chart_2 = ChartModule([ {"Label": "Total flow",
                       "Color": "Blue"},
                      {"Label": "Effective flow",
                       "Color": "Green"}],
                    data_collector_name='datacollector')

chart_3 = ChartModule([ {"Label": "Effective speed",
                       "Color": "Blue"},
                      {"Label": "Speed",
                       "Color": "Green"}],
                    data_collector_name='datacollector')

chart_4 = ChartModule([ {"Label": "Departure Delay",
                       "Color": "Blue"},
                      {"Label": "Enroute Delay",
                       "Color": "Green"},
                      {"Label": "Total Delay",
                       "Color": "Red"}],
                    data_collector_name='datacollector')

chart_5 = ChartModule([  {"Label": "N Arrivals",
                       "Color": "Blue"},
                       {"Label": "N Departures",
                       "Color": "Red"}],
                    data_collector_name='datacollector')

chart_7 = ChartModule([  {"Label": "N Conflicts",
                       "Color": "Blue"},
                       {"Label": "N Intrusions",
                       "Color": "Red"}],
                    data_collector_name='datacollector')

server = ModularServer(BoidFlockers, [boid_canvas, 
                                      chart_1,
                                      chart_2,
                                      chart_3,
                                      chart_4,
                                      chart_5,
                                      chart_7], "Boids", model_params)


