import json
import sys
from graphics import GraphWin
from Obstacle import Obstacle
from BoundingArea import BoundingArea
from KinematicPoint import KinematicPoint
from DynamicPoint import DynamicPoint
from DifferentialDrive import DifferentialDrive
from KinematicCar import KinematicCar
from Goal import Goal
from RRT import RRT


class Environment: 
    '''
    Environment represents takes care of running the
    models, generating obstacles and visualizing it.
    '''

    def __init__(self, obs, bounding, player, goal, win):
        '''
        m_type : [0,1,2,3] = [the models available]
        obs = list of lists of [x,y], each representing an obstacle
        bounding = list of [x,y] defining the bounding area
        '''
        self.win = win
        self.win.yUp()
        self.obstacles = [Obstacle(o,self.win) for o in obs]
        self.bounding_area = BoundingArea(bounding, self.win)
        self.player = player
        self.goal = goal
        self.rrt = None

    def gen_rrt(self, rrt_setup):
        self.rrt = RRT(self.bounding_area, self.obstacles, self.player, self.goal, 
            rrt_setup, self.win)
        self.rrt.generate()
        self.player.add_path(self.rrt.optimal_path)
        if isinstance(self.player, DynamicPoint):
            #TODO: check if of instance kin.car as well
            self.player.add_sling_path(self.goal)
        self.rrt.set_graphicals()
        #self.rrt.remove_graphicals()

    def run(self, rrt_setup):
        self.init_draw()
        self.gen_rrt(rrt_setup)
        while not self.player.finished:
            #self.player.set_auto_velocity(self.goal)
            self.player.set_velocity(self.goal)
        self.player.set_graphicals()
        print("Is finshed")
        print("Position:")
        print("goal: ", self.goal.pos_x,",",self.goal.pos_y)
        print("player: ", self.player.pos_x,",",self.player.pos_y)
        print("Velocity:")
        print("goal: ", self.goal.vel_x,",",self.goal.vel_y)
        print("player: ", self.player.vel_x,",",self.player.vel_y)
        print("Time taken to reach goal (sec): ", player.total_time)
        self.win.close()
        return player.total_time
    
    def init_draw(self):
         # draw everything initially
        self.bounding_area.set_graphicals()
        for obs in self.obstacles:
            obs.set_graphicals()
        self.player.set_graphicals()
        self.goal.set_graphicals()

if __name__ == "__main__":

    with open("best_strategies.json") as json_file:
        best_strategies = json.load(json_file)
    
    with open("current_strategy.json") as json_file:
        curr_strat = json.load(json_file)

    env_name = curr_strat['env_name']
    mdl_name = curr_strat['mdl_name']
    delta_q = curr_strat['delta_q']
    k = curr_strat['k']
    rrt_strat = curr_strat['rrt_strategy']


    with open("environments/"+str(env_name)+".json") as json_file:
        desc = json.load(json_file)
    
    bounding_poly = desc['bounding_polygon']
    obstacles = [desc[key] for key, val in desc.items() if key.startswith('obs')]
    pos_start = desc['pos_start']
    pos_goal = desc['pos_goal']
    vel_start = desc['vel_start']
    vel_goal = desc['vel_goal']
    dt = desc['vehicle_dt']
    v_max = desc['vehicle_v_max']
    a_max = desc['vehicle_a_max']
    omega_max = desc['vehicle_omega_max']
    phi_max = desc['vehicle_phi_max']
    vehicle_length = desc['vehicle_L']
    
    canvas_width = 800
    canvas_height = 800
    win = GraphWin("area", canvas_width, canvas_height)

    if mdl_name == 'kinematic_point':
        player = KinematicPoint(vel_start, pos_start, dt, v_max, win)
    elif mdl_name == "dynamic_point":
        player = DynamicPoint(vel_start, pos_start, dt, v_max, a_max, win)
    elif mdl_name == "differential_drive":
        player = DifferentialDrive(vel_start, pos_start, v_max, 0.00001, dt, win)
    elif mdl_name == "kinematic_car":
        player = KinematicCar(vel_start, pos_start, v_max, vehicle_length, phi_max, dt, win)
    else:
        print('Invalid model name, exiting')
        sys.exit(0)
    
    goal = Goal(vel_goal, pos_goal, win)
    env = Environment(obstacles, bounding_poly, player, goal, win)
    rrt_setup = {'delta_q': delta_q, 'k': k, 'strategy':rrt_strat, 'x_range': [-2,60], 'y_range': [-2,60]}
    new_time = env.run(rrt_setup)
    old_time = best_strategies[env_name][mdl_name]['best_time']
    resave = False
    if(old_time == -1):
        print("This time will be set as the new best time (no other recorded time")
        resave = True
    elif(new_time > old_time):
        print('This time was worse than the current best: ', old_time)
    elif(new_time < old_time):
        print('This time was better than the current best: ', old_time)
        print('Difference is: ', old_time - new_time)
        print('This time will be set as the new best time')
        resave = True
    if(resave):
        best_strategies[env_name][mdl_name] = {
            "best_time": new_time,
            "delta_q": delta_q,
            "k": k,
            "rrt_strategy": rrt_strat
        }
        with open('best_strategies.json', 'w') as json_file:
            json.dump(best_strategies, json_file)

