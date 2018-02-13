import json
import sys
import numpy as np
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

    def __init__(self, obs, bounding, player, goal, win, quick_draw=False):
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
        self.quick_draw = quick_draw

    def gen_rrt(self, rrt_setup):
        if isinstance(self.player, DynamicPoint) or isinstance(self.player, KinematicCar):
            good_rrt = False
            while not good_rrt:
                self.rrt = RRT(self.bounding_area, self.obstacles, self.player, self.goal, 
                rrt_setup, self.win, goal_rate=10)
                rrt_time = self.rrt.generate(data_keep=True)
                self.player.add_path(self.rrt.optimal_path)
                result = self.player.add_sling_path(self.goal, self.obstacles)
                if result != False:
                    good_rrt = True
        else:
            self.rrt = RRT(self.bounding_area, self.obstacles, self.player, self.goal, 
            rrt_setup, self.win, goal_rate=10)
            rrt_time = self.rrt.generate(data_keep=True)
            self.player.add_path(self.rrt.optimal_path)

        return rrt_time

    def run(self, rrt_setup):
        rrt_time = self.gen_rrt(rrt_setup)
        while not self.player.finished:
            self.player.set_velocity(self.goal)
        print("Time taken to reach goal (sec): ", player.total_time)
        return rrt_time, player.total_time
    

if __name__ == "__main__":
    
    with open("current_strategy.json") as json_file:
        curr_strat = json.load(json_file)

    env_name = curr_strat['env_name']
    mdl_name = curr_strat['mdl_name']
    delta_q = curr_strat['delta_q']
    k = curr_strat['k']
    rrt_strat = curr_strat['rrt_strategy']
    its = curr_strat['iterations']


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

    times = []
    rrt_times = []
    best_time = 1000
    worst_time =  0
    best_rrt_time = 1000
    worst_rrt_time = 0
    for i in range(its):
        if mdl_name == 'kinematic_point':
            player = KinematicPoint(vel_start, pos_start, dt, v_max, win)
        elif mdl_name == "dynamic_point":
            player = DynamicPoint(vel_start, pos_start, dt, v_max, a_max, win)
        elif mdl_name == "differential_drive":
            player = DifferentialDrive(vel_start, pos_start, v_max, omega_max, dt, win)
        elif mdl_name == "kinematic_car":
            player = KinematicCar(vel_start, pos_start, v_max, vehicle_length, phi_max, dt, win)
        else:
            print('Invalid model name, exiting')
            sys.exit(0)
        
        goal = Goal(vel_goal, pos_goal, win)
        env = Environment(obstacles, bounding_poly, player, goal, win, quick_draw=True)
        rrt_setup = {'delta_q': delta_q, 'k': k, 'strategy':rrt_strat, 'x_range': [-2,60], 'y_range': [-2,60]}
        rrt_time, time = env.run(rrt_setup)
        times.append(time)
        rrt_times.append(rrt_time)
        if time < best_time:
            best_time = time
        if time > worst_time:
            worst_time = time

        if rrt_time < best_rrt_time:
            best_rrt_time = rrt_time
        if rrt_time > worst_rrt_time:
            worst_rrt_time = rrt_time
    
    curr_strat['mean'] = np.mean(times)
    curr_strat['std_dev'] = np.std(times)
    curr_strat['worst'] = worst_time
    curr_strat['best'] = best_time
    curr_strat['mean_rrt'] = np.mean(rrt_times)
    curr_strat['std_dev_rrt'] = np.std(rrt_times)
    curr_strat['worst_rrt'] = worst_rrt_time
    curr_strat['best_rrt'] = best_rrt_time
    with open('experimental_results/rrt/1.0-'+str(env_name)+'-'+str(mdl_name)+'-'+str(its)+'-'+str(k)+'.json', 'w') as json_file:
            json.dump(curr_strat, json_file)
    print(np.mean(rrt_times), np.std(rrt_times), best_rrt_time, worst_rrt_time)
