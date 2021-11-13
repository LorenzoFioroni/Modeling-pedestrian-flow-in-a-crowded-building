import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp2d
import numpy.linalg as la

class Wall():
    def __init__(self, p1, p2, decay_length=1, intensity=1):
        self.p1 = p1
        self.p2 = p2
        self.R = decay_length
        self._v = (p2 - p1) / la.norm(p2-p1)**2
        self.I = intensity*la.norm(self.p2-self.p1)
        return

    def project(self, p):
        p = np.moveaxis(p, 0, 2)
        u = self.p1- p
        projections = np.empty_like(p)
        t = - np.tensordot(self._v, u, axes=[[0], [2]])
        filter = ((0 <= t) & (t <= 1))
        t = t[:,:, None]
        projections[filter] = (t*self.p2 + (1-t)*self.p1)[filter]
        nearest = np.argmin((
            la.norm(u, axis=2), 
            la.norm(self.p2-p, axis=2)
            ), axis=0)[:,:,None]
        projections[np.invert(filter)] = (
            self.p1 + (self.p2-self.p1)*nearest)[np.invert(filter)]
        return np.moveaxis(projections, 2, 0)

class Obstacle():
    def __init__(self, center, decay_length=1, intensity=1):
        self.p = center
        self.R = decay_length
        self.I = intensity
        return



class Environment():

    def __init__(self, discretization_length):
        self.compiled = False
        self.walls = []
        self.obstacles = []
        self.bounds = None
        self.dl = discretization_length
        return None
    
    def add_wall(self, wall):
        if self.compiled == True:
            raise Exception("The model has already been compiled")
        
        self.walls.append(wall)

        if (self.bounds is None):
            self.bounds = np.concatenate((
                np.min((wall.p1, wall.p2), axis=0),
                np.max((wall.p1, wall.p2), axis=0)
            ))
        else:
            self.bounds = np.concatenate((
                np.min((
                    self.bounds[0:2],
                    wall.p1,
                    wall.p2
                ), axis=0),
                np.max((
                    self.bounds[2:4],
                    wall.p1,
                    wall.p2
                ), axis=0)
                ))
    
    def add_obstacle(self, obstacle):
        if self.compiled == True:
            raise Exception("The model has already been compiled")
        
        self.obstacles.append(obstacle)

        if (self.bounds is None):
            self.bounds = np.concatenate((
                obstacle.p,
                obstacle.p
            ))
        else:
            self.bounds = np.concatenate((
                np.min((
                    self.bounds[0:2],
                    obstacle.p
                ), axis=0),
                np.max((
                    self.bounds[2:4],
                    obstacle.p
                ), axis=0)
                ))

    def compile(self):
        n = ((self.bounds[2:4] - self.bounds[0:2]) // self.dl).astype(int) + 1
        delta = (self.bounds[2:4] - self.bounds[0:2]) / n
        
        self.padded_bounds = np.concatenate((
            self.bounds[0:2]-delta/2,
            self.bounds[2:4]+delta/2
        ))

        

        self.x = np.linspace(self.bounds[0]-delta[0]/2, self.bounds[2]+delta[0]/2, n[0]+2)
        self.y = np.linspace(self.bounds[1]-delta[1]/2, self.bounds[3]+delta[1]/2, n[1]+2)
        self.grid = np.stack([*np.meshgrid(self.x, self.y, indexing="ij")])
        self.field = np.zeros_like(self.grid)

        for w in self.walls:
            self.field += w.I * (self.grid - w.project(self.grid)) / la.norm(self.grid - w.project(self.grid), axis=0)**2 * np.exp(-la.norm(self.grid - w.project(self.grid), axis=0)/w.R)
        for o in self.obstacles:
            self.field += o.I*(self.grid - o.p[:,None,None]) / la.norm(self.grid - o.p[:, None, None], axis=0)**2 * np.exp(-la.norm(self.grid - o.p[:, None, None], axis=0)/o.R)

        self.interp_x = interp2d(self.x,self.y, (self.field[0,:,:]).T)
        self.interp_y = interp2d(self.x,self.y, (self.field[1,:,:]).T)

        self.compiled = True

    def plot(self, plot_field = True, saturation_threshold = 5, plot_arrows = True, plot_grid = False, show = True):
        if (len(self.walls) == 0) and (len(self.obstacles) == 0):
            raise Exception("You should add at least one environmental item")

        fig = plt.figure()
        ax = fig.gca()
        ax.set_aspect('equal')

        expansion_factor = 0.08

        margin = (self.bounds[2:4] - self.bounds[0:2]) * expansion_factor

        if plot_field and self.compiled:
            margin = (self.padded_bounds[2:4] - self.padded_bounds[0:2]) * expansion_factor
            saturated_field = np.copy(self.field)

            if saturation_threshold is not None:
                saturated_field = np.moveaxis(saturated_field, 0, 2)
                filter = la.norm(saturated_field, axis=2) > saturation_threshold
                saturated_field[filter] = (saturated_field[filter].T / la.norm(saturated_field[filter], axis=1)*saturation_threshold).T
                saturated_field = np.moveaxis(saturated_field, 2, 0)

            plt.imshow(la.norm(saturated_field, axis=0).T, origin="lower", interpolation="bilinear", extent=[*self.padded_bounds[::2], *self.padded_bounds[1::2]])
            plt.colorbar()

            if plot_grid:
                plt.scatter(*self.grid, marker=".", color="orange")
            
            if plot_arrows:
                plt.quiver(*np.meshgrid(self.x, self.y, indexing="ij"), *saturated_field)

        for wall in self.walls:
            plt.plot([wall.p1[0], wall.p2[0]], [wall.p1[1], wall.p2[1]],
                     color="black", linewidth=1, marker=".")

        for obstacle in self.obstacles:
            plt.scatter(*obstacle.p, color="black", marker="x", linewidth=1)

        plt.xlim(self.padded_bounds[0]-margin[0], self.padded_bounds[2]+margin[0])
        plt.ylim(self.padded_bounds[1]-margin[1], self.padded_bounds[3]+margin[1])
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")

        if show:
            plt.show()
            return None
        else:
            return fig

    def get_field(self, position):
        if self.compiled == False: raise Exception("You have to compile the model first")
        return np.array([self.interp_x(*position), self.interp_y(*position)])

