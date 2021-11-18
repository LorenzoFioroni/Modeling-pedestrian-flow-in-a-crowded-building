import numpy as np
import matplotlib.pyplot as plt
import numpy.linalg as la


class Agent:

    ARRIVED = 1
    NOT_ARRIVED = 0

    def __init__(self, id, env, start_time, position, goal_position, desired_speed=np.random.normal(1.34, 0.26), relaxation_time = 0.5, V = 2.1,
                 sigma = 0.3):

        self.id = id
        self.env = env
        self.start_pos = np.copy(position)
        self.pos = [position]

        self.goal_position = goal_position

        self.relaxation_time = relaxation_time
        self.start_time = start_time

        self.desired_speed = desired_speed
        self.max_speed = 1.3 * self.desired_speed
        self.speed = [np.zeros(2)]

        self.V = V
        self.sigma = sigma
        self.fluctuaction_deviation = 15

        self.desired_direction = (self.goal_position - self.pos[-1]) / la.norm(self.goal_position - self.pos[-1])
        return None

    def compute_force(self, active_agents, delta):

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

        new_speed = self.speed[-1] + delta * self.compute_force(active_agents, delta)
        if la.norm(new_speed) != 0:
            if la.norm(new_speed) <= self.max_speed:
                self.speed.append(new_speed)
            else:
                self.speed.append(new_speed * self.max_speed / la.norm(new_speed))
        else:
            self.speed.append(np.zeros(2))

        if delta * la.norm(self.speed[-1]) >= la.norm(self.goal_position - self.pos[-1]) or la.norm(self.goal_position - self.pos[-1]) <= 0.2:
            self.pos.append(self.goal_position)
            self.end_time = current_time
            return Agent.ARRIVED

        else:
            self.pos.append(self.pos[-1] + delta * self.speed[-1])
            self.desired_direction = (self.goal_position - self.pos[-1]) / la.norm(self.goal_position - self.pos[-1])
            return Agent.NOT_ARRIVED

    def plot_trajectory(self, fig, show=True):

        trajectory = np.array(self.pos)

        plt.plot(trajectory[:, 0], trajectory[:, 1], label=self.id)

        if show:
            plt.show()
            return None
        else:
            return fig

