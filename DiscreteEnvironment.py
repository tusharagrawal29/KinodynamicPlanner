from __future__ import division
import numpy as np
import operator

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        print 'Dim', self.dimension, 'L', self.lower_limits, 'U', upper_limits, 'R', resolution
        for idx in range(self.dimension):
            self.num_cells[idx] = np.ceil((upper_limits[idx] - lower_limits[idx])/resolution[idx])

        # Create a fixed list instead of multiple calls to operator.mul
        self.multi_idx = [reduce(operator.mul, self.num_cells[0:i], 1) for i in range(self.dimension)]

    def ConfigurationToNodeId(self, config):

        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
        node_id = self.GridCoordToNodeId(self.ConfigurationToGridCoord(config))
        return int(node_id)

    def NodeIdToConfiguration(self, nid):

        # TODO:
        # This function maps a node in discrete space to a configuration
        # in the full configuration space
        #
        config = self.GridCoordToConfiguration(self.NodeIdToGridCoord(nid))
        return np.array(config)

    def ConfigurationToGridCoord(self, config):

        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        coord = [np.floor((config[i] - self.lower_limits[i])/self.resolution[i]) for i in range(0, len(config))]
        return np.array(coord)

    def GridCoordToConfiguration(self, coord):

        # TODO:
        # This function maps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [(coord[i] + 0.5)*self.resolution[i] + self.lower_limits[i] for i in range(0, len(coord))]
        return np.array(config)

    def GridCoordToNodeId(self, coord):

        # TODO:
        # This function maps a grid coordinate to the associated
        # node id
        node_id = 0
        for i in range(0, len(coord)):
            node_id += coord[i] * self.multi_idx[i]
        return int(node_id)

    def NodeIdToGridCoord(self, node_id):

        # TODO:
        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension
        for i in reversed(range(0, self.dimension)):
            coord[i] = np.floor(node_id/self.multi_idx[i])
            node_id = node_id % self.multi_idx[i]
        return np.array(coord)

