import numpy as np
from Environment import *
from Agent import *
import os
from itertools import chain
import warnings

warnings.filterwarnings("ignore", message="FixedFormatter should only be used together with FixedLocator")
from datetime import timedelta


class Simulation():

    def __init__(self, env, time_step):

        self.time = 0
        self.dt = time_step
        self.env = env
        self.active_agents = []
        self.inactive_agents = []
        self.sleeping_agents = []
        self.agent_number = 0
        self.arrived_number = [0]

        if not self.env.compiled:
            self.env.compile()

    def add_agent(self, agent):
        if agent.id is None:
            agent.id = f"a{self.agent_number + 1}"
        self.sleeping_agents.append(agent)
        self.agent_number += 1

    def move(self, replace=False):

        self.arrived_number.append(self.arrived_number[-1])

        for a in range(len(self.active_agents)):

            agent = self.active_agents.pop(0)

            if agent.move(self.active_agents, self.dt, self.time) == Agent.ARRIVED:
                self.inactive_agents.append(agent)
                self.arrived_number[-1] += 1
                if replace:
                    self.add_agent(Agent(
                        self.env,
                        start_time=self.time,
                        position=agent.pos[0],
                        goal_position_start=agent.goal_position_start,
                        goal_position_end=agent.goal_position_end,
                        desired_speed=agent.desired_speed,
                        relaxation_time=agent.relaxation_time,
                        V=agent.V,
                        sigma=agent.sigma
                    ))
            else:
                self.active_agents.append(agent)

    def run(self, t_max=np.inf, replace=False):

        while (self.agent_number - len(self.inactive_agents) != 0 and self.time <= (t_max - self.dt)):

            for i in range(len(self.sleeping_agents)):
                a = self.sleeping_agents.pop(0)
                if a.start_time <= self.time:
                    self.active_agents.append(a)
                else:
                    self.sleeping_agents.append(a)

            self.move(replace)
            self.time += self.dt

    def plot(self, plot_field=True, plot_arrows=False, plot_grid=False, plot_agents=True, show=True):

        fig = self.env.plot(plot_field=plot_field, plot_arrows=plot_arrows, plot_grid=plot_grid, show=False)
        if plot_agents:
            for a in self.inactive_agents:
                fig = a.plot_trajectory(fig, show=False)
        plt.legend()
        if show:
            plt.show()
        else:
            return fig

    def save_frames(self, plot_field=True, plot_arrows=False, plot_grid=False,
                    path=os.path.join(os.getcwd(), "animation")):

        N = int(self.time / self.dt)
        if not (os.path.exists(path) and os.path.isdir(path)):
            os.makedirs(path)
        pattern = os.path.join(path, "frame_*.png")
        os.system(f"rm {pattern}")
        print(f"Saving the frames of the animation in folder {path}")

        for i, t in tqdm(enumerate(np.linspace(0, self.time, N)), total=N):
            fig = self.env.plot(plot_field=plot_field, plot_arrows=plot_arrows, plot_grid=plot_grid, show=False)
            for a in chain(self.inactive_agents, self.active_agents):

                zero = int(np.floor(a.start_time / self.dt))
                if 0 <= i - zero < len(a.pos):
                    plt.scatter(*a.pos[i - zero], label=f"Agent {a.id}")

            plt.legend()
            isec, fsec = divmod(round(t * 100), 100)
            plt.title(f"{timedelta(seconds=isec)}.{fsec:02.0f}")
            plt.savefig(os.path.join(path, f"frame_{i}.png"))
            plt.close(fig)

        pattern = os.path.join(path, "frame_%01d.png")
        target = os.path.join(path, "Animation.mp4")
        print(
            "All the frames have been generated. You can merge them in a video using ffmpeg with the following command:")
        command = f"ffmpeg -framerate {int(1 / self.dt)} -i {pattern} -y {target}"
        print(command)
        print("Do you want to run it? y/N")
        if input() == "y":
            os.system(command)

    def mean_TimeToGoal(self, normalize=True):

        mean = 0
        if normalize:
            for a in self.inactive_agents:
                mean += (a.end_time - a.start_time) - \
                        self.optimal_TimeToGoal(
                            Agent(a.id, self.env, start_time=0, position=a.pos[0], goal_position=a.goal_position,
                                  desired_speed=a.desired_speed, relaxation_time=a.relaxation_time,
                                  V=a.V, sigma=a.sigma))
        else:
            for a in self.inactive_agents:
                mean += (a.end_time - a.start_time)

        return mean / len(self.inactive_agents)

    def optimal_TimeToGoal(self, agent):

        env_local = Simulation(self.env, [agent], self.delta)
        env_local.run()
        return env_local.inactive_agents[0].end_time
