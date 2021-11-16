import numpy as np
import numpy.linalg as la
import matplotlib.pyplot as plt

class Agent:

    def __init__(self, start_time, position, relaxation_time, max_speed, goal_position, desired_speed, V):

        self.start_pos = np.copy(position)
        self.pos = [position]

        self.goal_position = goal_position

        self.relaxation_time = relaxation_time
        self.start_time = start_time

        self.max_speed = max_speed
        self.desired_speed = desired_speed
        self.speed = [np.zeros(2)]

        self.V

        self.desired_direction = (self.goal_position - self.pos[-1]) / la.norm(self.goal_position - self.pos[-1])

    def compute_force(self, env , active_agents, delta):

        self.F = (self.desired_speed * self.desired_direction - self.speed) / self.relaxation_time
        self.F += env.compute_fields(self.pos[-1])

        for agent in active_agents:

            v = la.norm(agent.speed[-1])
            r = self.position[-1] - agent.position[-1]
            norm_r = la.norm(r)
            norm_r_ = la.norm(r - v * agent.desired_direction * delta)
            b = np.sqrt((norm_r + norm_r_) ** 2 - v ** 2) / 2

            self.F += r * (2 + (norm_r_ / norm_r) + (norm_r / norm_r_)) * np.exp(-b / agent.sigma) * self.V / (b * agent.sigma * 4)

        return self.F

    def move(self, env, active_agents, delta, current_time):

            new_speed = self.speed[-1] + delta * self.compute_force(env, active_agents, delta)
            if la.norm(new_speed) != 0:
                if la.norm(new_speed) <= self.max_speed:
                    self.speed.append(new_speed)
                else:
                    self.speed.append(new_speed * self.max_speed / la.norm(new_speed))
            else:
                self.speed.append(np.zeros(2))

            if delta * la.norm(self.speed[-1]) >= la.norm(self.goal_position-self.pos[-1]):
                self.pos.append(self.goal_position)
                self.end_time = current_time
                return "Arrived"

            else:
                self.pos.append( self.pos[-1] + delta * self.speed[-1])
                self.desired_direction = (self.goal_position - self.pos[-1]) / la.norm(self.goal_position - self.pos[-1])
                return "Not_Arrived"

    def plot_trajectory(self, env):

        trajectory = np.array(self.pos)
        plt.axis([np.min(trajectory[:, 0]) - 1, np.max(trajectory[:, 0]) + 1,
                  np.min(trajectory[:, 1]) - 1, np.max(trajectory[:, 1]) + 1])
        plt.plot(trajectory[:, 0], trajectory[:, 1], linestyle="dashed")

        if len(env.Obstacles) > 0:
            plt.scatter(env.Obstacles[0].p[0], env.Obstacles[0].p[1], label="Obstacles", c="g")
            for obst in env.Obstacles[1:]:
                plt.scatter(obst.p[0], obst.p[1], c="g")

        if len(env.Walls) > 0:
            plt.plot(env.Walls[0].p1, env.Walls[0].p2, label="Walls", c="k")
            for wall in env.Walls[1:]:
                plt.plot(wall.p1, wall.p2, c="k")

        plt.scatter(self.start_pos[0], self.start_pos[1], label="Start", c="r", marker="v", s=80)
        plt.scatter(self.goal_position[0], self.goal_position[1], label="End", c="r", marker="^", s=80)
        plt.legend()

class env():

    def add_wall(self, wall):

    def add_wall_from_segments(self, start, end, refinement):
        #start has to be a N by 2 np.array with N being the numbers of segments used
        N = start.shape[0]
        for i in range(N):
            x_d = np.hstack((np.arange(start[i, 0], end[i, 0], refinement * 0.5), end[i, 0]))
            y_d = np.hstack((np.arange(start[i, 1], end[i, 1], refinement * 0.5), end[i, 1]))
            for j in range(x_d.shape[0]):
                self.add_wall()

    def add_wall_from_curve(self, fun, start, end, refinement):

        #fun has to be a 1-dim parametrization of the curve defining the wall, and provided a set of N points return
        # a N by 2 np array
        d = fun(np.hstack((np.arange(start, end, refinement), end)))
        for i in range(d.shape[0]):
            self.add_wall()