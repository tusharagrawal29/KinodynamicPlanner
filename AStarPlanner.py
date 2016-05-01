from heapq import heappop, heappush, heapify
import time
import numpy as np

class AStarPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def Plan(self, start_config, goal_config):
        init_time = time.time()

        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of way-points
        #  and n is the dimension of the robots configuration space
        # start_config = [0, -3.65, 0]
        # goal_config = [2, -3.65, 0]
        print 'start', start_config, 'goal', goal_config
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)

        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        expand = []
        self.nodes[start_id] = None
        motion_cost = {start_id: 0}
        plan = []
        i = start_id
        while True:
            successors = self.planning_env.GetSuccessors(i)
            for (j, j_action) in successors:
                cost = self.planning_env.ComputeDistance(i, j) + motion_cost[i]
                if j not in self.nodes:
                    self.nodes[j] = (i, j_action)
                    motion_cost[j] = cost
                    priority = cost + self.planning_env.ComputeHeuristicCost(j, goal_id)
                    heappush(expand, (priority, j))
                    if self.visualize:
                        self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(i), self.planning_env.discrete_env.NodeIdToConfiguration(j))

                elif cost < motion_cost[j]:
                    self.nodes[j] = (i, j_action)
                    motion_cost[j] = cost
                    priority = cost + self.planning_env.ComputeHeuristicCost(j, goal_id)
                    entry = next((i for i, v in enumerate(expand) if v[1] == j), None)
                    if entry is not None:
                        expand[entry] = (priority, j)
                        heapify(expand)
                    if self.visualize:
                        self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(i), self.planning_env.discrete_env.NodeIdToConfiguration(j))
            if goal_id in motion_cost:
                print "found"
                break
            i = heappop(expand)[1]

        while not goal_id == None and not self.nodes[goal_id] == None:
            print goal_id
            (goal_id, action) = self.nodes[goal_id]
            plan.append(action)

        plan = plan[::-1]
        # plan.insert(0, start_config)
        # plan.append(goal_config)

        # sum = 0

        # for i in range(1, len(plan)):
        #     sum += np.linalg.norm(plan[i] - plan[i-1])

        # print "Time: ",time.time() - init_time," Length of Path: ", sum, " No. of Nodes: ", len(self.nodes)
        # print plan
        return plan
