import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp2d
import numpy.linalg as la
from tqdm import tqdm

class Wall():
    """ A wall is an environmental item defined as a segment in the plane.

    Arguments:
    p1 = numpy array of shape (2,) containing the coordinates of the first point
    p2 = numpy array of shape (2,) containing the coordinates of the second point
    decay_length = characteristic length used to compute the force field due to a 
        specific instance of the wall (default is 1)
    intensity = scale factor used to compute the force field due to a specific 
        instance of the wall (default is 1)
    """

    def __init__(self, p1, p2, decay_length=1, intensity=1):
        self.p1 = p1
        self.p2 = p2
        self.R = decay_length
        self.v = (p2 - p1) / la.norm(p2-p1)**2
        self.I = intensity*la.norm(self.p2-self.p1)
        return

    def project(self, p):
        """ Projects a set of points onto the segment representing the wall.
            
        Arguments:
        p = numpy array of shape (2, N, M) with arbitrary N, M. 

        Returns a numpy array of shape (2, N, M) where the value at [:, i, j] is 
        the projection of p[:, i, j] onto the segment representing the wall.
        """
        p = np.moveaxis(p, 0, 2)
        u = self.p1- p
        projections = np.empty_like(p)
        t = - np.tensordot(self.v, u, axes=[[0], [2]])
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
    """ An obstacle is an environmental item defined as a point in the plane.

    Arguments:
    center = numpy array of shape (2,) containing the coordinates of the obstacle 
    decay_length = characteristic length used to compute the force field due to a 
        specific instance of the obstacle (default is 1)
    intensity = scale factor used to compute the force field due to a specific 
        instance of the obstacle (default is 1)
    """

    def __init__(self, center, decay_length=1, intensity=1):
        self.p = center
        self.R = decay_length
        self.I = intensity
        return



class Environment():
    """ Class defining the environment in which the agents are going to move. 
    Provides the add_wall, add_obstacle, add_wall_from_polygonal, add_wall_from_curve
    functions to populate the environment, the function compile to compute the
    force field due to the added environmental items, a function plot for 
    visualization and a method get_field to get the force field in a specific point.
    The field is computed on a discrete grid and then the values are interpolated.

    Arguments:
    discretization_length = the step size of the grid used to compute the force field
    """

    def __init__(self, discretization_length):
        self.compiled = False
        self.walls = []
        self.obstacles = []
        self.bounds = None
        self.dl = discretization_length
        return None
    
    def add_wall(self, wall):
        """ If the model has not been compiled yet, adds a wall to the environment.
            
        Arguments:
        wall = the instance of the class Wall to be added
        """
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
        """ If the model has not been compiled yet, adds an obstacle to the environment.
            
        Arguments:
        obstacle = the instance of the class Obstacle to be added
        """
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
        """ Computes the force field due to the environmental items added.
        Generates a grid with step size self.discretization_length and computes 
        the field on the points of the grid. Two interpolator models are the 
        set up to retrive the value of the field in an arbitrary point in the plane.
        """
        print("Compiling the model...")

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

        for w in tqdm(self.walls):
            self.field += w.I * (self.grid - w.project(self.grid)) / la.norm(self.grid - w.project(self.grid), axis=0)**2 * np.exp(-la.norm(self.grid - w.project(self.grid), axis=0)/w.R)
        for o in tqdm(self.obstacles):
            self.field += o.I*(self.grid - o.p[:,None,None]) / la.norm(self.grid - o.p[:, None, None], axis=0)**2 * np.exp(-la.norm(self.grid - o.p[:, None, None], axis=0)/o.R)

        self.interp_x = interp2d(self.x[:],self.y[:], (self.field[0,:,:]).T,kind="quintic")
        self.interp_y = interp2d(self.x[:],self.y[:], (self.field[1,:,:]).T,kind="quintic")

        print("The model has successfully been compiled")

        self.compiled = True

    def plot(self, plot_field = True, saturation_threshold = 5, plot_arrows = True, plot_grid = False, show = True):
        """ Plots the environment
            
        Arguments:
        plot_field = whether to plot the force field intensity as an heatmap
        saturation_threshold = the value to which the field saturates (only 
            affects visualization)
        plot_arrows = whether to draw the field as a vector
        plot_grid = whether to plot the grid where the field has been computed
        show = whether to show the plot

        If show is set to true, returns None. Otherwise returns the figure instance
        """
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
            cbar = plt.colorbar()
            cbar.set_label("Force intensity")
            fig.canvas.draw()
            labels = cbar.ax.get_yticklabels()
            labels[-1].set_text(f"$\geq${labels[-1].get_text()}")
            cbar.ax.set_yticklabels(labels)

            if plot_grid:
                plt.scatter(*self.grid, marker=".", color="orange")
            
            if plot_arrows:
                plt.quiver(*np.meshgrid(self.x, self.y, indexing="ij"), *saturated_field)

        for wall in self.walls:
            plt.plot([wall.p1[0], wall.p2[0]], [wall.p1[1], wall.p2[1]],
                     color="black", linewidth=1, marker=".")

        for obstacle in self.obstacles:
            plt.scatter(*obstacle.p, color="black", marker="x", linewidth=1)

        if self.compiled:
            plt.xlim(self.padded_bounds[0]-margin[0], self.padded_bounds[2]+margin[0])
            plt.ylim(self.padded_bounds[1]-margin[1], self.padded_bounds[3]+margin[1])
        else:
            plt.xlim(self.bounds[0]-margin[0], self.bounds[2]+margin[0])
            plt.ylim(self.bounds[1]-margin[1], self.bounds[3]+margin[1])

        plt.xlabel("x [m]")
        plt.ylabel("y [m]")

        if show:
            plt.show()
            return None
        else:
            return fig

    def get_field(self, position):
        """ If the model has already been compiled, gets the field in a specific 
            point interpolating between the points on the grid
            
        Arguments:
        position = a numpy array of shape (2,) containing the coordinates of the 
            points where the field should be retrived
        
        Returns a numpy array of shape (2,) containing the field in the given point
        """

        if self.compiled == False: raise Exception("You have to compile the model first")
        return np.array([self.interp_x(*position)[0], self.interp_y(*position)[0]])

    def add_wall_from_polygonal(self, vertices, step_length, decay_length= None, intensity= None):
        """ If the model has not been compiled yet, adds a set of walls defined
            by the vertices of a polygonal chain.
            
        Arguments:
        vertices = a numpy array of shape (N, 2), with N arbitrary. Between each
            pair of consecutive points is the added a wall.
        step_length = the desired lenght of the walls. This step size is then 
            adjusted in a way that the walls between two vertices have the same length
        decay_length = characteristic length used to compute the force field due to a 
            specific instance of the wall (If None, the default value from the 
            Wall class will be used. default is None.)
        intensity = scale factor used to compute the force field due to a specific 
            instance of the wall (If None, the default value from the Wall class 
            will be used. default is None.)
        """
        if self.compiled == True:
            raise Exception("The model has already been compiled")

        args = {}
        if decay_length != None: args["decay_length"] = decay_length
        if intensity != None: args["intensity"] = intensity

        for i in range(len(vertices)-1):
            N = int(la.norm(vertices[i+1]-vertices[i]) // step_length)
            x = np.linspace(vertices[i, 0], vertices[i+1, 0], N)
            y = np.linspace(vertices[i, 1], vertices[i+1, 1], N)
            for i in range(N-1):
                self.add_wall(Wall(np.array([x[i], y[i]]), np.array([x[i+1], y[i+1]]), **args))

    def add_wall_from_curve(self, func, start, end, step_length, decay_length= None, intensity= None):
        """ If the model has not been compiled yet, adds a set of walls defined
            by 1D parametrization of a curve.
            
        Arguments:
        func = a function taking in input a numpy array with shape (N,) with 
            arbitrary N, and returning a numpy array of shape (2, N).
        start = the start point for the parameter defining the curve
        end = the end point for the parameter defining the curve
        step_length = the step size for the parameter defining the curve. This step
            size is then adjusted in a way that the values of the parameter are 
            equally spaced between start and end
        decay_length = characteristic length used to compute the force field due to a 
            specific instance of the wall (If None, the default value from the 
            Wall class will be used. default is None.)
        intensity = scale factor used to compute the force field due to a specific 
            instance of the wall (If None, the default value from the Wall class 
            will be used. default is None.)
        """
        if self.compiled == True:
            raise Exception("The model has already been compiled")

        args = {}
        if decay_length != None: args["decay_length"] = decay_length
        if intensity != None: args["intensity"] = intensity

        N = int((end-start) // step_length)
        s = func(np.linspace(start,end,N)).T
        for i in range(s.shape[0]-1):
            self.add_wall(Wall(s[i], s[i+1], **args))