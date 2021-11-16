import numpy as np 
from Environment import *
from Agent import *

env = Environment(discretization_length=0.1)
x = np.linspace(0, 2*np.pi, 50)
for i in range(len(x)-1):
    env.add_wall(
        Wall(np.array([x[i], np.sin(x[i])+0.5*x[i]+1]), np.array([x[i+1], np.sin(x[i+1])+0.5*x[i+1]+1])))
    env.add_wall(
        Wall(np.array([x[i], np.sin(x[i])+0.5*x[i]-1]), np.array([x[i+1], np.sin(x[i+1])+0.5*x[i+1]-1])))
x = np.linspace(-1,1, 20)
for i in range(len(x)-1):
    env.add_wall(Wall(np.array([0, x[i]]), np.array([0, x[i+1]])))
    env.add_wall(Wall(np.array([2*np.pi, x[i]+np.pi]), np.array([2*np.pi, x[i+1]+np.pi])))

env.compile()

a1 = Agent("a", env, 0, np.array([0.3, 0]), 0.5, 5, np.array([2*np.pi-0.3, np.pi]), 1 , 1, 1)
a2 = Agent("b", env, 0, np.array([4, 0.8]), 0.5, 5, np.array([1.98, 2.47]), 1 , 1, 1)
a3 = Agent("c", env, 0, np.array([0.3, 0]), 0.5, 5, np.array([2*np.pi-0.3, np.pi]), 1 , 1, 1)


t = 0
dt = 0.01
while(t<20):
    a1.move([a2],dt,t)
    a2.move([a1],dt,t)
    a3.move([], dt, t)
    t+=dt

fig = env.plot(plot_field=True, plot_arrows=False, plot_grid=False, show=False)

fig  = a1.plot_trajectory(fig, show=False)
fig  = a3.plot_trajectory(fig, show=False)
fig  = a2.plot_trajectory(fig, show=False)
plt.legend()
plt.show()

# In this example a and b move according the the social force model to reach their goal positions. c starts at the same position of a and has the same goal. In its evolution, however, is neglected the presence of the other agents. c's trajectory allows one to compare the trajectories in presence/absence of other agents