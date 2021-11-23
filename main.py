import numpy as np
import matplotlib.pyplot as plt
from Environment import * 
from Agent import *
from Simulation import *

env = Environment(discretization_length=0.1)

env.add_wall_from_polygonal(np.array([[0, 4], [0, 0], [10, 0], [10,4]]), step_length=0.1)
env.add_wall_from_polygonal(np.array([[10, 6], [10, 10], [0, 10], [0,6]]), step_length=0.1)
env.compile()
# env.plot()

sim = Simulation(env, 0.1)

for i in range(50):
    if np.random.rand()<0.5:
        target = np.array([0, np.random.uniform(4.5, 5.5)])
        start = np.array([10, np.random.uniform(4.5, 5.5)])
    else:
        target = np.array([10, np.random.uniform(4.5, 5.5)])
        start = np.array([0, np.random.uniform(4.5, 5.5)])
    sim.add_agent(Agent(env, np.abs(np.random.normal(loc=3, scale=1.5)), start, target))

sim.run(t_max = 60)

print(f"Mean time to goal: {sim.mean_TimeToGoal(normalize=False)}")

# sim.plot(plot_field = False)

sim.save_frames(plot_field=False)