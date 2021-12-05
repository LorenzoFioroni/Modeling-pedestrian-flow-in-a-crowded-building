import numpy as np
from Environment import *
from Agent import *
import os
from itertools import chain
import warnings
from datetime import timedelta
warnings.filterwarnings("ignore", message="FixedFormatter should only be used together with FixedLocator")


class Simulation():
    """ Class accounting for the whole simulation.
    Provides the add_agent function to add agents to the simulation
    If the provided environment is not compiled, it will be compiled
    automatically. The functions move and run implement the actual
    evolution of the system. with plot and save_frames one can visualize
    the simulated motion of the agents.
    Arguments:
    env = the environment in which the agents are going to move 
    time_step = the timestep for the simulation 
    """

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
        """ Adds an agent to the simulation
        Arguments:
        agent = the instance of the class Agent to be added
        """

        if agent.id is None:
            agent.id = f"a{self.agent_number + 1}"
        self.sleeping_agents.append(agent)
        self.agent_number += 1

    def move(self):
        """ Executes one step of the simulation
        """

        self.arrived_number.append(self.arrived_number[-1])

        for a in range(len(self.active_agents)):

            agent = self.active_agents.pop(0)

            if agent.move(self.active_agents, self.dt, self.time) == Agent.ARRIVED:
                self.inactive_agents.append(agent)
                self.arrived_number[-1] += 1
            else:
                self.active_agents.append(agent)

    def run(self, t_max=np.inf):
        """ Runs the simulation from time 0 to either t_max or
        until all the agents arrived to their target positions.
        Arguments:
        t_max = the time limit of the simulation (Default is infinite).
        """

        print("Running the simulation...")
        with tqdm(bar_format="Simulation time: {n:0.2f}s \t [{elapsed}s, {rate_fmt}{postfix}]") as pbar:
            while (self.agent_number - len(self.inactive_agents) != 0 and self.time <= (t_max - self.dt)):

                for i in range(len(self.sleeping_agents)):
                    a = self.sleeping_agents.pop(0)
                    if a.start_time <= self.time:
                        self.active_agents.append(a)
                    else:
                        self.sleeping_agents.append(a)
                self.move()
                self.time += self.dt
                pbar.update(self.dt)
        print()

    def plot(self, plot_field=True, plot_arrows=False, plot_grid=False, plot_agents=True, show=True):
        """ Plots the the trajectories of each agent in the simulation.
        Arguments:
        plot_field = whether to plot the force field intensity as an heatmap
        plot_arrows = whether to draw the field as a vector
        plot_grid = whether to plot the grid where the field has been computed
        plot_agents = whether to plot the agents 
        show = whether to show the plot
        If show is set to true, returns None. Otherwise returns the figure instance
        """

        fig = self.env.plot(plot_field=plot_field, plot_arrows=plot_arrows, plot_grid=plot_grid, show=False)
        if plot_agents:
            for a in self.inactive_agents:
                fig = a.plot_trajectory(fig, show=False)
        if show:
            plt.show()
        else:
            return fig

    def save_frames(self, plot_field=True, plot_arrows=False, plot_grid=False, path=os.path.join(os.getcwd(), "animation")):
        """ Saves the frames of the simulation. After all the frames have
            been saved, it's prompted whether to merge them in a video using ffmpeg
        Arguments:
        plot_field = whether to plot the force field intensity as an heatmap
        plot_arrows = whether to draw the field as a vector
        plot_grid = whether to plot the grid where the field has been computed
        path = the path where to save the frames. Default is 
            {{current working directory}}/animation
        """

        N = int(self.time / self.dt)
        if not (os.path.exists(path) and os.path.isdir(path)):
            os.makedirs(path)
        pattern = os.path.join(path, "frame_*.png")
        os.system(f"rm {pattern}")
        print(f"\nSaving the frames of the animation in folder {path}")

        fig = self.env.plot(plot_field=plot_field, plot_arrows=plot_arrows, plot_grid=plot_grid, show=False)
        xy = []
        ax = fig.gca()
        points = plt.scatter([], [], animated=True)
        bg = fig.canvas.copy_from_bbox(fig.bbox)
        fig.canvas.blit(fig.bbox)
        fig.canvas.draw()
        for i, t in tqdm(enumerate(np.linspace(0, self.time, N)), total=N):
            fig.canvas.restore_region(bg)
            xy = []
            for a in chain(self.inactive_agents, self.active_agents):
                zero = int(np.floor(a.start_time / self.dt))
                if 0 <= i - zero < len(a.pos):
                    xy.append([*a.pos[i - zero]])
                    points.set_offsets(xy)
                    ax.draw_artist(points)
            isec, fsec = divmod(round(t * 100), 100)
            plt.title(f"{timedelta(seconds=isec)}.{fsec:02.0f}")
            plt.savefig(os.path.join(path, f"frame_{i}.png"))

        pattern = os.path.join(path, "frame_%01d.png")
        target = os.path.join(path, "Animation.mp4")
        print("All the frames have been generated. You can merge them in a video using ffmpeg with the following command:")
        command = f"ffmpeg -framerate {int(1 / self.dt)} -i {pattern} -y -c:v libx264 -c:a aac -strict experimental -tune fastdecode -pix_fmt yuv420p {target}"
        print(f">\t{command}")
        print("Do you want to run it? y/N")
        if input() == "y":
            os.system(command)

    def mean_TimeToGoal(self):
        """ Computes the average over all the agents in the simulation
            of the time needed to reach their goal position.
            Returns a float representing that average
        """

        mean = 0
        for a in self.inactive_agents:
            mean += (a.end_time - a.start_time)

        return mean / len(self.inactive_agents)
