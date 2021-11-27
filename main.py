import numpy as np
import matplotlib.pyplot as plt
from Environment import * 
from Agent import *
from Simulation import *


# Definition of the environment

env = Environment(discretization_length=0.1)

env.add_wall_from_polygonal(np.array([[0, 4], [0, 0], [10, 0], [10,4]]), step_length=0.1)
env.add_wall_from_polygonal(np.array([[10, 6], [10, 10], [0, 10], [0,6]]), step_length=0.1)
env.compile()
# env.plot()


# Definition of the simulation

sim = Simulation(env, 0.1)


# Adding agents to the simulation

for i in range(50):
    if np.random.rand()<0.5:
        target0 = np.array([0, 4.1])
        target1 = np.array([0, 5.9])
        start = np.array([10, np.random.uniform(4.1, 5.9)])
    else:
        target0 = np.array([10, 4.1])
        target1 = np.array([10, 5.9])
        start = np.array([0, np.random.uniform(4.1, 5.9)])
    sim.add_agent(Agent(env, np.abs(np.random.uniform(0, 10)), start, target0, target1))


# Running the simulation 

sim.run(t_max = 60)

print(f"Mean time to goal: {sim.mean_TimeToGoal(normalize=False)}")

# sim.plot(plot_field = False)

# Generating animation frames

sim.save_frames(plot_field=False)