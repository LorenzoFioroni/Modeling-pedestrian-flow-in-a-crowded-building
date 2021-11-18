import numpy as np
import matplotlib.animation as animation
from Environment import *
from Agent import *

class Simulation():

    def __init__(self, env, active_agents, delta):

        self.time = 0
        self.delta = delta
        self.env = env
        self.active_agents = active_agents
        self.inactive_agents = []
        self.agent_number = int(len(active_agents))
        self.arrived_number = [0]

        if not self.env.compiled:
            self.env.compile()

    def optimal_TimeToGoal(self, agent):

        env_local = Simulation(self.env, [agent], self.delta)
        env_local.move_to_goal()
        return env_local.inactive_agents[0].end_time

    def add_agents(self, position, goal_position, desired_speed=np.random.normal(1.34, 0.26), relaxation_time = 0.5, V = 2.1,
                 sigma = 0.3):

        self.active_agents.append(Agent("a" + str(self.agent_number + 1), self.env, self.time,
                                        position, goal_position, desired_speed=np.random.normal(1.34, 0.26),
                                        relaxation_time=0.5, V=2.1,
                                        sigma=0.3))
        self.agent_number += 1

    def move(self, replace=False):

        self.arrived_number.append(self.arrived_number[-1])
        for o in range(len(self.active_agents)):

            agent = self.active_agents[0]
            del self.active_agents[0]

            if agent.move(self.active_agents, self.delta, self.time):
                self.inactive_agents.append(agent)
                self.arrived_number[-1] += 1
                if replace:
                    self.active_agents.append(Agent(agent.id, self.env, start_time=self.time, position=agent.pos[0],
                                                    goal_position=agent.goal_position, desired_speed=agent.desired_speed,
                                                    relaxation_time=agent.relaxation_time, V=agent.V, sigma=agent.sigma))

            else:
                self.active_agents.append(agent)

    def move_to_goal(self, t_max=np.inf, replace=False):

        while (len(self.active_agents) != 0 and self.time <= (t_max - self.delta)):
            self.move(replace)
            self.time += self.delta

    def plot(self, plot_field=True, plot_arrows=False, plot_grid=False, plot_agents=True, show=True):

        fig = self.env.plot(plot_field=True, plot_arrows=False, plot_grid=False, show=False)
        if plot_agents:
            for a in self.inactive_agents:
                fig = a.plot_trajectory(fig, show=False)
        plt.legend()
        if show:
            plt.show()
        else:
            return fig

    def save_frames(self, plot_field=True, plot_arrows=False, plot_grid=False, plot_agents=True):

        N = int(self.time / self.delta)
        for i in range(N):
            fig = self.env.plot(plot_field=True, plot_arrows=False, plot_grid=False, show=False)
            for a in self.inactive_agents:
                if i < len(a.pos):
                    plt.scatter(a.pos[i][0], a.pos[i][1], label="agent " + a.id)
                else:
                    plt.scatter(a.goal_position[0], a.goal_position[1], label="agent " + a.id)
            plt.legend()
            plt.savefig("Frame/frame" + str(i) + ".png", )
            plt.close(fig)
            print("Frame at time " + str(i * self.delta) + "/" + str(self.time) + " saved.")

    def mean_TimeToGoal(self, normalize=True):

        mean = 0
        if normalize:
            for a in self.inactive_agents:
                mean += (a.end_time - a.start_time) - \
                        self.optimal_TimeToGoal(Agent(a.id, self.env, start_time=0, position=a.pos[0], goal_position=a.goal_position,
                                                      desired_speed=a.desired_speed, relaxation_time=a.relaxation_time,
                                                      V=a.V, sigma=a.sigma))
        else:
            for a in self.inactive_agents:
                mean += (a.end_time - a.start_time)

        return mean / len(self.inactive_agents)

