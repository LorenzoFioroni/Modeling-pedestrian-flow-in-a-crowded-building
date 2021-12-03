import numpy as np
import matplotlib.pyplot as plt
from Environment import * 
from Agent import *
from Simulation import *


# Definition of the environment

env = Environment(discretization_length=0.1)

mode=[4, 5]
## fst int: number of exits                     base case: 1
## snd int: obstacle
##              0 - no obstacles            <-  base case
##              1 - grid
##              2 - grid_shifted
##              3 - pillars_infrontof_exit
##              4 - random_obstacles
##              5 - wedge

## walls ##
if mode[0] != 4:
    ## 2 exits ##
    env.add_wall_from_polygonal(np.array([[0, 4], [0, 0], [10, 0], [10,4]]), step_length=0.1)
    env.add_wall_from_polygonal(np.array([[10, 6], [10, 10], [0, 10], [0,6]]), step_length=0.1)
else:
    ## 4 exits ##
    env.add_wall_from_polygonal(np.array([[0, 4], [0, 0], [4, 0]]), step_length=0.1)
    env.add_wall_from_polygonal(np.array([[6, 0], [10, 0], [10, 4]]), step_length=0.1)
    env.add_wall_from_polygonal(np.array([[10, 6], [10, 10], [6, 10]]), step_length=0.1)
    env.add_wall_from_polygonal(np.array([[4, 10], [0, 10], [0, 6]]), step_length=0.1)

## obstacles ##
if mode[1] == 1:
    ## grid ##
    for i in range(2, 9):
        for j in range(2, 9):
            env.add_obstacle(Obstacle(np.array([i, j])))
elif mode[1] == 2:
    ## grid_shifted ##
    for i in range(2, 9):
        for j in range(2, 9):
            if j%2==0:
                env.add_obstacle(Obstacle(np.array([i, j])))
            elif i<8:
                env.add_obstacle(Obstacle(np.array([i+.5, j])))
elif mode[1] == 3:
    ## pillars_infrontof_exit ##
    env.add_obstacle(Obstacle(np.array([2, 5])))
    env.add_obstacle(Obstacle(np.array([8, 5])))
    if mode[0] == 4:
        env.add_obstacle(Obstacle(np.array([5, 2])))
        env.add_obstacle(Obstacle(np.array([5, 8])))

elif mode[1] == 4:
    ## random_obstacles ##
    for i in range(50):
        env.add_obstacle(Obstacle(np.array([np.random.uniform(1, 9), np.random.uniform(1, 9)])))
elif mode[1] == 5:
    ## wedge ##
    env.add_wall_from_polygonal(np.array([[5, 6], [3, 5], [5, 4], [7, 5], [5, 6]]), step_length=0.1, intensity=2.0)

env.compile()
# env.plot()


# Definition of the simulation

sim = Simulation(env, 0.1)


# Adding agents to the simulation

for i in range(50):
    if mode[0] == 2:
    ## 2 exits ##
        if np.random.rand()<0.5:
            target0 = np.array([10, 4.1])
            target1 = np.array([10, 5.9])
            start = np.array([0, np.random.uniform(4.1, 5.9)])
        else:
            target0 = np.array([0, 4.1])
            target1 = np.array([0, 5.9])
            start = np.array([10, np.random.uniform(4.1, 5.9)])
    elif mode[0] == 4:
    ## 4 exits ##
        r = np.random.rand()
        if r < 0.25:
            target0 = np.array([10, 4.1])
            target1 = np.array([10, 5.9])
            start = np.array([0, np.random.uniform(4.1, 5.9)])
        elif r < 0.5:
            target0 = np.array([4.1, 10])
            target1 = np.array([5.9, 10])
            start = np.array([np.random.uniform(4.1, 5.9), 0])
        elif r < 0.75:
            target0 = np.array([0, 4.1])
            target1 = np.array([0, 5.9])
            start = np.array([10, np.random.uniform(4.1, 5.9)])
        else:
            target0 = np.array([4.1, 0])
            target1 = np.array([5.9, 0])
            start = np.array([np.random.uniform(4.1, 5.9), 10])
    else:
    ## 1 exit ##
        target0 = np.array([10, 4.1])
        target1 = np.array([10, 5.9])
        start = np.array([0, np.random.uniform(4.1, 5.9)])
    sim.add_agent(Agent(env, np.abs(np.random.uniform(0, 10)), start, target0, target1))


# Running the simulation 

sim.run(t_max = 60)

print(f"Mean time to goal: {sim.mean_TimeToGoal(normalize=False)}")
print(f"Times of arrival: {sim.arrived_number}")

sim.plot(plot_field = False)
sim.plot(plot_agents = False)

# Generating animation frames

sim.save_frames(plot_field=False)