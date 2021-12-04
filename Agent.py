import numpy as np
import matplotlib.pyplot as plt
import numpy.linalg as la
import warnings
warnings.filterwarnings("ignore", message="FixedFormatter should only be used together with FixedLocator")

class Agent:
    """ Class defining the behaviour of each agent in the simulation.
    Provides the proj_to_goal and compute_force functions to account for
    the environment where he is moving and the move function to perform an 
    actual step of the simulation. The the function plot_trajectory plots
    the path along which the agent has moved so far.
    Arguments:
    env = the environment in which the agent is going to move
    start_time = the time the agent joins the simulation
    position = the initial position of the agent
    goal_position_start = the goal position of the agent is defined as a
        segment (e.g. a door). Here is specified the first end of the segment
    goal_position_end = the goal position of the agent is defined as a
        segment (e.g. a door). Here is specified the second end of the segment
    desired_speed = the ideal speed for the specific agent. Default is a random
        speed distributed gaussian around 1.34, with a standard deviation of 0.26
    relaxation_time = characteristic time which determines how quickly an agent 
        changes his speed. Default is 0.5
    V = a parameter used to compute the forces due to other agents. Default is 2.1
    sigma = a parameter used to compute the forces due to other agents. Default is 0.3
    id = a label for the specific agent. If None, the label "unknown" will be used.
        Default is None
    """
    ARRIVED = 1
    NOT_ARRIVED = 0

    def __init__(self, env, start_time, position, goal_position_start, goal_position_end, desired_speed=np.random.normal(1.34, 0.26), relaxation_time = 0.5, V = 2.1, sigma = 0.3, id=None):

        self.id = id
        self.env = env
        self.start_pos = np.copy(position)
        self.pos = [position]

        self.goal_position_start = goal_position_start
        self.goal_position_end = goal_position_end

        if goal_position_start[0] != goal_position_end[0]:
            self.goal_coeff = [(self.goal_position_end[1] - self.goal_position_start[1]) /
                               (self.goal_position_end[0] - self.goal_position_start[0])]
            self.goal_coeff.append(self.goal_position_end[1] - self.goal_position_end[0] * self.goal_coeff[0])

        self.relaxation_time = relaxation_time
        self.start_time = start_time

        self.desired_speed = desired_speed
        self.max_speed = 1.3 * self.desired_speed
        self.speed = [np.zeros(2)]

        self.V = V
        self.sigma = sigma
        self.fluctuaction_deviation = 30

        self.desired_direction = (self.proj_to_goal() - self.pos[-1]) / la.norm(self.proj_to_goal() - self.pos[-1])
        return None

    def proj_to_goal(self):
        """ Projects self's position onto the segment representing the target.
        Returns a numpy array representing the projection of self's position
        onto the segment representing the wall.
        """

        if self.goal_position_start[0] != self.goal_position_end[0]:
            x = (self.pos[-1][0] + self.goal_coeff[0] * (self.pos[-1][1] - self.goal_coeff[1])) / \
                (1 + self.goal_coeff[0] ** 2)
            if x < min(self.goal_position_start[0], self.goal_position_end[0]):
                x = min(self.goal_position_start[0], self.goal_position_end[0])
            if x > max(self.goal_position_start[0], self.goal_position_end[0]):
                x = max(self.goal_position_start[0], self.goal_position_end[0])
            return np.array([x, self.goal_coeff[0] * x + self.goal_coeff[1]])

        if self.goal_position_start[0] == self.goal_position_end[0]:
            x = self.pos[-1][1]
            if x < min(self.goal_position_start[1], self.goal_position_end[1]):
                x = min(self.goal_position_start[1], self.goal_position_end[1])
            if x > max(self.goal_position_start[1], self.goal_position_end[1]):
                x = max(self.goal_position_start[1], self.goal_position_end[1])
            return np.array([self.goal_position_start[0], x])

    def compute_force(self, active_agents, delta):
        """ Computes the force to which the agent is subject.
        Arguments:
        active_agents = a list containing all the other agents active in the 
            simulation
        delta = the timestep dt
        Returns the resultant force on the agent
        """

        self.F = (self.desired_speed * self.desired_direction - self.speed[-1]) / self.relaxation_time
        self.F += self.env.get_field(self.pos[-1])

        for agent in active_agents:
            v = la.norm(agent.speed[-1])
            v_ = v * agent.desired_direction * delta
            r = self.pos[-1] - agent.pos[-1]
            norm_r = la.norm(r)
            norm_r_ = la.norm(r - v_)
            b = np.sqrt((norm_r + norm_r_) ** 2 - (v * delta) ** 2) / 2

            self.F += (r * (2 + (norm_r_ / norm_r) + (norm_r / norm_r_)) - v_ * (1 + (norm_r / norm_r_))) \
                      * np.exp(-b / agent.sigma) * self.V / (b * agent.sigma * 4)


        theta = np.radians(np.random.normal(scale=self.fluctuaction_deviation))
        self.F = np.dot(np.array(((np.cos(theta), -np.sin(theta)), (np.sin(theta), np.cos(theta)))), self.F)

        return self.F

    def move(self, active_agents, delta, current_time):
        """ Computes the new position and velocity of the agent after one 
        step of the simulation. 
        Arguments:
        active_agents = a list containing all the other agents active in the 
            simulation
        delta = the timestep dt
        current_time = the current simulation time
        """
        new_speed = self.speed[-1] + delta * self.compute_force(active_agents, delta)
        if la.norm(new_speed) != 0:
            if la.norm(new_speed) <= self.max_speed:
                self.speed.append(new_speed)
            else:
                self.speed.append(new_speed * self.max_speed / la.norm(new_speed))
        else:
            self.speed.append(np.zeros(2))

        #if delta * la.norm(self.speed[-1]) >= la.ngoal_position_endorm(self.goal_position - self.pos[-1]) or la.norm(self.goal_position - self.pos[-1]) <= 0.2:
        if la.norm(self.pos[-1] - self.proj_to_goal()) <= delta * la.norm(self.speed[-1]):
            self.pos.append(self.proj_to_goal())
            self.end_time = current_time
            return Agent.ARRIVED

        else:
            self.pos.append(self.pos[-1] + delta * self.speed[-1])
            self.desired_direction = (self.proj_to_goal() - self.pos[-1]) / la.norm(self.proj_to_goal() - self.pos[-1])
            return Agent.NOT_ARRIVED

    def plot_trajectory(self, fig, show=True):
        """ Plots the path walked by the agent
        Arguments:
        fig = the pyplot figure object where to draw the trajectory
        show = whether to show the plot. Default is True
        If show is set to true, returns None. Otherwise returns the figure instance
        """

        trajectory = np.array(self.pos)

        plt.plot(trajectory[:, 0], trajectory[:, 1], label=self.id)
        if self.goal_position_start[0] != self.goal_position_end[0]:
            plt.plot([self.goal_position_start[0], self.goal_position_end[0]],
                     [self.goal_position_start[1], self.goal_position_end[1]], c="r")

        if show:
            plt.show()
            return None
        else:
            return fig


    def __repr__(self):
        return self.id or "unknown"
