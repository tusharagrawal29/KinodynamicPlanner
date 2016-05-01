import numpy as np
from DiscreteEnvironment import DiscreteEnvironment
from random import uniform, randint
import time

class HerbEnvironment(object):

    def __init__(self, herb, resolution):

        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)
        self.env = self.robot.GetEnv()

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # self.weights = np.array(self.robot.GetActiveDOFWeights())

        self.weights = np.array([2.8, 1.8, 1.6, 1.4, 1.2, 1, 0.8])

        # add a table and move the robot into place
        self.table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')

        self.robot.GetEnv().Add(self.table)

        table_pose = np.array([[ 0, 0, -1, 0.7],
                                  [-1, 0,  0, 0],
                                  [ 0, 1,  0, 0],
                                  [ 0, 0,  0, 1]])
        self.table.SetTransform(table_pose)

        # set the camera
        camera_pose = np.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)

    def GetID(self, axis, change, grid_coord):
        grid_coord[axis] += change
        return self.discrete_env.GridCoordToNodeId(grid_coord)

    def collision_check(self, x):
        with self.env:
            self.robot.SetDOFValues(x, self.robot.GetActiveDOFIndices(), True)
        return self.env.CheckCollision(self.robot, self.table) or self.robot.CheckSelfCollision()

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())
        #
        # TODO: Generate and return a random configuration
        #
        while True:
            config = [uniform(self.lower_limits[i], self.upper_limits[i]) for i in range(0, len(self.lower_limits))]
            if self.collision_check(config):
                pass
            else:
                break
        return np.array(config)

    def Extend(self, start_config, end_config, goal=False):

        #
        # TODO: Implement a function which attempts to extend from
        #   a start configuration to a goal configuration
        #
        if goal:
            steps = max(5, np.linalg.norm(start_config - end_config)*10)
        else:
            steps = np.linalg.norm(start_config - end_config)*10
        config_temp = [np.linspace(start_config[i], end_config[i], steps) for i in range(0, len(self.lower_limits))]
        config_temp = np.array(zip(*config_temp))
        for i in range(1, len(config_temp)):
            if self.collision_check(config_temp[i]):
                return None
            if self.ComputeDistance_RRT(start_config, config_temp[i]) > 0.75:
                return config_temp[i]
        return end_config

    def GetSuccessors(self, node_id):

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes

        gridcoord = self.discrete_env.NodeIdToGridCoord(node_id)
        successors = []
        for i in range(0, len(self.lower_limits)):
            for j in range(-1, 2, 2):
                temp = gridcoord[i] + j
                if 0 <= temp <= self.discrete_env.num_cells[i]:
                    candidate = self.GetID(i, j, list(gridcoord))
                    if not self.collision_check(self.discrete_env.NodeIdToConfiguration(candidate)):
                        successors.append(candidate)
        return successors

    def ComputeDistance_RRT(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        with self.env:
            self.robot.SetDOFValues(end_config, self.robot.GetActiveDOFIndices(), True)
        t1 = np.array(self.robot.GetActiveManipulator().GetTransform())
        with self.env:
            self.robot.SetDOFValues(start_config, self.robot.GetActiveDOFIndices(), True)
        t2 = np.array(self.robot.GetActiveManipulator().GetTransform())
        return np.linalg.norm(t1[:, 3] - t2[:, 3])

    def ComputeHeuristicCost_RRT(self, start_config, goal_config):

        # TODO: Here you will implement a function that
        # computes the heuristic cost between the configurations

        return sum(self.weights*np.absolute(np.array(goal_config) - np.array(start_config)))

    def ComputeDistance(self, start_id, end_id):

        # TODO: Here you will implement a function that
        # computes the distance between the configurations given
        # by the two node ids
        with self.env:
            self.robot.SetDOFValues(self.discrete_env.NodeIdToConfiguration(end_id), self.robot.GetActiveDOFIndices(), True)
        t1 = np.array(self.robot.GetActiveManipulator().GetTransform())
        with self.env:
            self.robot.SetDOFValues(self.discrete_env.NodeIdToConfiguration(start_id), self.robot.GetActiveDOFIndices(), True)
        t2 = np.array(self.robot.GetActiveManipulator().GetTransform())
        return np.linalg.norm(t1[:, 3] - t2[:, 3])

    def ComputeHeuristicCost(self, start_id, goal_id):

        # TODO: Here you will implement a function that
        # computes the heuristic cost between the configurations
        # given by the two node ids

        return sum(self.weights*np.absolute(np.array(self.discrete_env.NodeIdToConfiguration(goal_id)) - np.array(self.discrete_env.NodeIdToConfiguration(start_id))))

    def ShortenPath(self, path, timeout=5.0):
        #
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the
        #  given timout (in seconds).
        #

        # make copy so we don't overwrite the original path
        short_path = list(path)

        init_time = time.time()
        while time.time() - init_time < timeout:
            start = randint(0, len(short_path)-3)
            end = randint(start+1, len(short_path)-1)
            extend = self.Extend(short_path[start], short_path[end])
            if np.array_equal(extend, short_path[end]):
                short_path[start+1:end] = []

        return short_path
