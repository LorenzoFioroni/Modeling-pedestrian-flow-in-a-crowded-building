import numpy as np
import matplotlib.pyplot as plt
from Environment import * 
from Agent import *
from Simulation import *

VERTICAL_MODE = 0
HORIZONTAL_MODE = 1
U_MODE = 2

mode = HORIZONTAL_MODE

# Definition of the environment

env = Environment(discretization_length=0.1)

env.add_wall_from_polygonal(np.array([[7, 8], [7, 8.5], [0, 8.5], [0,0], [7,0],[7,6]]), step_length=0.1, intensity=2, decay_length=1.3)


if mode == VERTICAL_MODE:
    deltaDesk = np.array([[-0.75,-0.35], [0.75, -0.35], [0.75, 0.35], [-0.75, 0.35], [-0.75,-0.35]])
    x = (np.array([0.5*(3.5-0.75), 3.5, 7-0.5*(3.5-0.75)])) - 0.01
    y = ((0.7+0.8)*np.arange(4)+0.35+0.8) - 0.01
    for x_coo in x:
        for y_coo in y:
            env.add_wall_from_polygonal(deltaDesk + np.array([x_coo, y_coo]), step_length=0.1, decay_length=1.5, intensity=1.5)

elif mode == HORIZONTAL_MODE:
    deltaDesk = np.array([[-0.35,-0.75], [0.35, -0.75], [0.35, 0.75], [-0.35, 0.75], [-0.35,-0.75]])
    y = np.arange(0, 4, 1)*(8.5/5-4*1.5/5+1.5)+8.5/5-4*1.5/5+0.75 -0.01
    x = ((0.7+0.8)*np.arange(3)+0.35+0.8) - 0.01
    for x_coo in x:
        for y_coo in y:
            env.add_wall_from_polygonal(deltaDesk + np.array([x_coo, y_coo]), step_length=0.1, decay_length=1.5, intensity=1.2)

elif mode == U_MODE:
    verticalDesk = np.array([[-0.35,-0.75], [0.35, -0.75], [0.35, 0.75], [-0.35, 0.75], [-0.35,-0.75]])
    horizontalDesk = np.array([[-0.75,-0.35], [0.75, -0.35], [0.75, 0.35], [-0.75, 0.35], [-0.75,-0.35]])
    env.add_wall_from_polygonal(horizontalDesk + np.array([2.25, 1.15]), step_length=0.1, decay_length=1.5, intensity=1.2)
    env.add_wall_from_polygonal(horizontalDesk + np.array([3.75, 1.15]), step_length=0.1, decay_length=1.5, intensity=1.2)
    env.add_wall_from_polygonal(horizontalDesk + np.array([5.25, 1.15]), step_length=0.1, decay_length=1.5, intensity=1.2)
    env.add_wall_from_polygonal(verticalDesk + np.array([1.15, 1.55]), step_length=0.1, decay_length=1.5, intensity=1.2)
    env.add_wall_from_polygonal(verticalDesk + np.array([1.15, 3.05]), step_length=0.1, decay_length=1.5, intensity=1.2)
    env.add_wall_from_polygonal(horizontalDesk + np.array([2.25, 3.45]), step_length=0.1, decay_length=1.5, intensity=1.2)
    env.add_wall_from_polygonal(horizontalDesk + np.array([2.25, 5.05]), step_length=0.1, decay_length=1.5, intensity=1.2)
    env.add_wall_from_polygonal(verticalDesk + np.array([1.15, 5.45]), step_length=0.1, decay_length=1.5, intensity=1.2)
    env.add_wall_from_polygonal(verticalDesk + np.array([1.15, 6.95]), step_length=0.1, decay_length=1.5, intensity=1.2)
    env.add_wall_from_polygonal(horizontalDesk + np.array([5.25, 7.35]), step_length=0.1, decay_length=1.5, intensity=1.2)
    env.add_wall_from_polygonal(horizontalDesk + np.array([3.75, 7.35]), step_length=0.1, decay_length=1.5, intensity=1.2)
    env.add_wall_from_polygonal(horizontalDesk + np.array([2.25, 7.35]), step_length=0.1, decay_length=1.5, intensity=1.2)

env.compile()
# env.plot()

# Definition of the simulation
sim = Simulation(env, 0.05)

# Adding agents to the simulation
if mode == VERTICAL_MODE:
    for x_coo in x:
        for y_coo in y:
            sim.add_agent(Agent(env, 0, np.array([x_coo, y_coo])+np.array([-0.5, -0.5]), np.array([7, 6]), np.array([7, 8]),sigma=0.7))
            sim.add_agent(Agent(env, 0, np.array([x_coo, y_coo])+np.array([0.5, -0.5]), np.array([7, 6]), np.array([7, 8]),sigma=0.7))
elif mode == HORIZONTAL_MODE:
    for x_coo in x:
        for y_coo in y:
            sim.add_agent(Agent(env, 0, np.array([x_coo, y_coo])+np.array([-0.5, -0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([x_coo, y_coo])+np.array([-0.5, 0.5]), np.array([7, 6]), np.array([7, 8])))
elif mode == U_MODE:
            sim.add_agent(Agent(env, 0, np.array([2.25, 1.15])+np.array([-0.5, -0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([2.25, 1.15])+np.array([0.5, -0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([3.75, 1.15])+np.array([-0.5, -0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([3.75, 1.15])+np.array([0.5, -0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([5.25, 1.15])+np.array([-0.5, -0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([5.25, 1.15])+np.array([0.5, -0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([1.15, 1.55])+np.array([-0.5, -0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([1.15, 1.55])+np.array([-0.5, 0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([1.15, 3.05])+np.array([-0.5, -0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([1.15, 3.05])+np.array([-0.5, 0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([2.25, 3.45])+np.array([-0.5, 0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([2.25, 3.45])+np.array([0.5, 0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([2.25, 5.05])+np.array([-0.5, -0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([2.25, 5.05])+np.array([0.5, -0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([1.15, 5.45])+np.array([-0.5, -0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([1.15, 5.45])+np.array([-0.5, 0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([1.15, 6.95])+np.array([-0.5, -0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([1.15, 6.95])+np.array([-0.5, 0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([5.25, 7.35])+np.array([-0.5, 0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([5.25, 7.35])+np.array([0.5, 0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([3.75, 7.35])+np.array([-0.5, 0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([3.75, 7.35])+np.array([0.5, 0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([2.25, 7.35])+np.array([-0.5, 0.5]), np.array([7, 6]), np.array([7, 8])))
            sim.add_agent(Agent(env, 0, np.array([2.25, 7.35])+np.array([0.5, 0.5]), np.array([7, 6]), np.array([7, 8])))

# Running the simulation 

sim.run(t_max=600)

print(f"Mean time to goal: {sim.mean_TimeToGoal()}")
print(f"Times of arrival: {sim.arrived_number}")

sim.plot(plot_agents = False, show=False)
plt.tight_layout()
plt.savefig("h_field.png")
plt.close()
sim.plot(plot_field = False, show=False)
plt.tight_layout()
plt.savefig("h_trajectory.png")
plt.close()

# Generating animation frames

sim.save_frames(plot_field=False, path=os.path.join(os.getcwd(),"animation"))