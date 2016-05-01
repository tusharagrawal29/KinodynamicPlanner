import numpy, openravepy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        self.lower_limits, self.upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        self.resolution = resolution
        self.ConstructActions()
        # self.PlotActionFootprints(0)

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        # Initialize the footprint
        config = start_config.copy()
        footprint = [numpy.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance
                
            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*numpy.array([xdot, ydot, tdot])
            if config[2] > numpy.pi:
                config[2] -= 2.*numpy.pi
            if config[2] < -numpy.pi:
                config[2] += 2.*numpy.pi

            footprint_config = config.copy()
            footprint_config[:2] -= start_config[:2]
            footprint.append(footprint_config)

            timecount += stepsize
            
        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)

        return footprint

    def PlotActionFootprints(self, idx):

        actions = self.actions[idx]
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        
        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            pl.plot(xpoints, ypoints, 'k')
                     
        pl.ion()
        pl.show()

        

    def ConstructActions(self):

        # Actions is a dictionary that maps orientation of the robot to
        #  an action set
        self.actions = dict()
              
        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)
        # print 'construct actions'
        # Iterate through each possible starting orientation
        for idx in range(int(self.discrete_env.num_cells[2])):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = self.discrete_env.GridCoordToConfiguration(grid_coordinate)

            # TODO: Here you will construct a set of actions
            #  to be used during the planning process
            #
            omega_rs = numpy.linspace(-1, 1, 5)
            omega_ls = numpy.linspace(-1, 1, 5)
            duration_list = numpy.linspace(0,3,8)
            for o_l in omega_ls:
                for o_r in omega_rs:
                    for d in duration_list:
                        control = Control(o_l, o_r, d)
                        footprint = self.GenerateFootprintFromControl(start_config, control)
                        action = Action(control, footprint)
                        self.actions[idx].append(action)
            # print idx
            # omega_r = 1 if start_config[2] < numpy.pi/2 else -1 # TODO set based on the value? Kp?
            # omega_l = -omega_r
            # tdot = self.herb.wheel_radius * (omega_l - omega_r) / self.herb.wheel_distance
            
    def CheckCollision(self, config):
        original_H = self.robot.GetTransform()
        H = numpy.identity(4)
        angle = config[2]
        H[0:2,0:2] = numpy.array([[numpy.cos(angle), -numpy.sin(angle)], [numpy.sin(angle), numpy.cos(angle)]])
        H[0:2, 3] = [config[0], config[1]]
        with self.robot.GetEnv():
            self.robot.SetTransform(H)
        ret = self.robot.GetEnv().CheckCollision(self.robot)
        with self.robot.GetEnv():
            self.robot.SetTransform(original_H)
        return ret


    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes
        gridcoord = self.discrete_env.NodeIdToGridCoord(node_id)
        config = self.discrete_env.GridCoordToConfiguration(gridcoord)
        lower_limits, upper_limits = self.boundary_limits
        actions = self.actions[gridcoord[2]]
        for action in actions:
            fp = action.footprint[-1]
            new_config = numpy.add(config, fp)
            new_config[2] = fp[2]
            if (lower_limits <= new_config).all() and  (new_config<= upper_limits).all() and not self.CheckCollision(new_config):
                Nid = self.discrete_env.ConfigurationToNodeId(new_config)
                successors.append((Nid, action))
        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)        
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
        dist = numpy.linalg.norm(start_config-end_config) 
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        # cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)        
        end_config = self.discrete_env.NodeIdToConfiguration(goal_id)
        xy_dist = numpy.linalg.norm(start_config[0:2]- end_config[0:2]) 
        return xy_dist;
