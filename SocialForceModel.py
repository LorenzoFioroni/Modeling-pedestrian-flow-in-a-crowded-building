import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy.linalg as la
plt.rcParams.update({'font.size':20})

def proj (wllw):

    walker = wllw[1]
    wall = wllw[0]

    if not wall.a == "inf":
        x_proj = (walker.pos[0] + wall.a * (walker.pos[1] - wall.b)) / (1 + wall.a ** 2)
        if x_proj >= wall.start and x_proj <= wall.end:
            return np.array([x_proj, wall.a * x_proj + wall.b])
        if x_proj > wall.end:
            return np.array([wall.end, wall.a * wall.end + wall.b])
        if x_proj < wall.start:
            return np.array([wall.start, wall.a * wall.start + wall.b])

    else:
        x_proj = wall.b
        if walker.pos[1] >= wall.start and walker.pos[1] <= wall.end:
            return np.array([x_proj, walker.pos[1]])
        if x_proj > walker.pos[1]:
            return np.array([x_proj, wall.end])
        if x_proj < walker.pos[1]:
            return np.array([x_proj, wall.start])

class Env():

    def __init__(self, Obstacles, Walls):
        self.name = "toto"
        self.Obstacles = Obstacles
        self.N_obst = len(self.Obstacles)
        self.Walls = Walls
        self.N_walls = len(self.Walls)

class Obstacle():

    # for now obstacle is just a point
    def __init__(self, pos, R):
        self.pos = pos
        self.R = R

class Wall():

    def __init__(self, a, b, start, end, R):
        self.a = a
        self.b = b
        self.start = start
        self.end = end
        self.R = R

class Force():

    def __init__(self, type="will"):
        if type == "will":
            self.ev = lambda walker: (walker.desired_speed * walker.desired_direction - walker.speed) / walker.relaxation_time
        if type == "obst":
            #self.ev = lambda ow: np.exp(- la.norm(ow[1].pos - ow[0].pos) / ow[0].R) * (ow[1].pos - ow[0].pos) / (ow[0].R * la.norm(ow[1].pos - ow[0].pos))
            self.ev = lambda ow: ow[0].R * (ow[1].pos - ow[0].pos) / la.norm(ow[1].pos - ow[0].pos) ** (ow[0].R + 2)
        if type == "wall":
            self.ev = lambda wllw: wllw[0].R * (wllw[1].pos - proj(wllw)) / la.norm(wllw[1].pos - proj(wllw)) ** (wllw[0].R + 2)
        if type == "walker":
            self.ev = lambda : np.zeros(2)

class Crowd():

    def __init__(self, Walkers, relaxation_time):
        #walkers is a list of walker
        self.Walkers = Walkers
        self.N = len(Walkers)

class Walker():

    def __init__(self, position, relaxation_time, max_speed, goal_position, desired_speed, delta, c, angle_vision):

        self.start_pos = np.copy(position)
        self.pos = position
        self.goal_position = goal_position

        self.relaxation_time = relaxation_time
        self.t = 0
        self.delta = delta

        self.max_speed = max_speed
        self.desired_speed = desired_speed
        self.speed = np.zeros(2)
        self.old_speed = np.zeros(2)

        self.c = c
        self.angle_vision= angle_vision

        self.desired_direction = (self.goal_position - self.pos) / la.norm(self.goal_position - self.pos)
        self.force_will = Force("will")

    def update_speed(self, new_speed):

        if la.norm(new_speed) !=0:
            if la.norm(new_speed) <= self.max_speed:
                self.speed = new_speed
            else:
                self.speed = new_speed * self.max_speed / la.norm(new_speed)
        else:
            self.speed = np.zeros(2)

    def env_recognition(self, U_obst, U_wall, U_walkers):

        self.U_obst = U_obst
        self.force_obst = Force("obst")
        self.U_wall = U_wall
        self.force_wall = Force("wall")
        self.U_walkers = U_walkers
        self.force_walker = Force("walker")

    def vision(self, force):

        if np.inner(self.desired_direction, force) >= la.norm(force) * np.cos(self.angle_vision):
            return 1
        else:
            return self.c

    def move(self, env, Walkers):

        if la.norm(self.pos - self.goal_position) != 0:

            self.F = self.force_will.ev(self)
            print("Before:", self.F)
            i=0
            for obst in env.Obstacles:
                force = self.force_obst.ev([obst, self])
                self.F += self.U_obst[i] * force * self.vision(force)
                i += 1
            i=0
            for wall in env.Walls:
                force = self.force_wall.ev([wall, self])
                self.F += self.U_wall[i] * force * self.vision(force)
                i+=1
            for walker in Walkers:
                force = 0
            print("After:", self.F)
            self.old_speed = self.speed
            self.speed = self.speed + self.delta * self.F
            self.update_speed(self.speed)

            if self.delta * la.norm(self.speed) >= la.norm(self.goal_position-self.pos):
                self.pos = self.goal_position
                self.desired_direction = np.zeros(2)
                self.speed = np.zeros(2)

            else:
                self.pos += self.delta * self.old_speed
                self.desired_direction = (self.goal_position - self.pos) / la.norm(self.goal_position - self.pos)

            self.t += self.delta

    def move_to_goal(self, env, Walkers):

        self.x_traj = [walker1.pos[0]]
        self.y_traj = [walker1.pos[1]]

        while walker1.pos[1] != self.goal_position[1] and walker1.pos[0] != self.goal_position[0]:

            walker1.move(env, Walkers)
            self.x_traj.append(walker1.pos[0])
            self.y_traj.append(walker1.pos[1])

    def plot_trajectory(self, env):

        plt.axis([np.min(self.x_traj) - 1, np.max(self.x_traj) + 1, np.min(self.y_traj) - 1, np.max(self.y_traj) + 1])
        plt.plot(self.x_traj, self.y_traj, linestyle="dashed")
        for obst in env.Obstacles:
            plt.scatter(obst.pos[0], obst.pos[1], label="Obstacles")
        for wall in env.Walls:
            pts = np.linspace(wall.start, wall.end, 100)
            if not wall.a == "inf":
                plt.plot(pts, wall.a * pts + wall.b, label="Walls", c="k")
            else:
                plt.plot(wall.b * np.ones(100), pts, label="Walls", c="k")
        plt.scatter(self.start_pos[0], self.start_pos[1], label="Start", c="r", marker="v",s=80)
        plt.scatter(self.goal_position[0], self.goal_position[1], label="End", c="r", marker="^", s=80)
        plt.legend()


walker1 = Walker(position=np.array([0., 1.]), relaxation_time=0.06, max_speed=1.3 * 1.3, desired_speed=1.3, delta=0.01
                 , goal_position=np.array([3, 3]), c=0.5, angle_vision=1.7453292519943295)

obst1 = Obstacle(pos=np.array([1, 0.95]), R=2)
obst2 = Obstacle(pos=np.array([1.1, 1.95]), R=2)
obst3 = Obstacle(pos=np.array([1.5, 2]), R=2)

wall1 = Wall(a=0., b=0.5, start=0., end=3.5, R=2.)
wall2 = Wall(a=0., b=1.5, start=0., end=2.5, R=2.)
wall3 = Wall(a="inf", b=2.5, start=1.5, end=4, R=2.)
wall4 = Wall(a="inf", b=3.5, start=0.5, end=4, R=2.)

env = Env([], [wall1, wall2, wall3, wall4])

walker1.env_recognition([], [0.1, 0.1, 0.1, 0.1], [])
walker1.move_to_goal(env, [])
walker1.plot_trajectory(env)




