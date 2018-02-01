import json
from graphics import *
import numpy as np
from Obstacle import Obstacle
from BoundingArea import BoundingArea
from KinematicPoint import KinematicPoint
from DynamicPoint import DynamicPoint
from Goal import Goal

class Environment: 
    '''
    Environment represents takes care of running the
    models, generating obstacles and visualizing it.
    '''

    def __init__(self, obs, bounding):
        '''
        m_type : [0,1,2,3] = [the models available]
        obs = list of lists of [x,y], each representing an obstacle
        bounding = list of [x,y] defining the bounding area
        '''
        canvas_width = 800
        canvas_height = 600
        self.win = GraphWin("area", canvas_width, canvas_height)
        self.win.yUp()
        self.obstacles = [Obstacle(o,self.win) for o in obs]
        self.bounding_area = BoundingArea(bounding, self.win)
        self.player = None
        self.goal = None

    def calculate_path(self):
        #rrt = RRT(self.bounding_area, self.obstacles, self.player, self.goal)
        a=2

    def set_player(self, type, vel, pos, dt, v_max, **kwargs):
        if type == 0:
            self.player = KinematicPoint(vel, pos, dt, v_max, self.win)
        if type == 1:
            self.player  = DynamicPoint(vel, pos, dt, v_max, kwargs['acc_max'], self.win)

    def set_goal(self, vel, pos):
        self.goal = Goal(vel, pos, self.win)

    def show(self):
        
        # draw everything initially
        self.bounding_area.set_graphicals()
        for obs in self.obstacles:
            obs.set_graphicals()

        self.goal.set_graphicals()
        self.player.set_graphicals()

        while not self.player.finished:
            self.player.set_velocity(self.goal)
            self.player.move()
        print("Is finshed")
        self.win.getMouse()
        self.win.close()

if __name__ == "__main__":
    #run stuff here
    with open("P1.json") as json_file:
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
    env = Environment(obstacles, bounding_poly)
    # run for kinematic point
    #env.set_player(0, vel_start, pos_start, dt, v_max)
    # run for dynamic point
    env.set_player(1, vel_start, pos_start, dt, v_max, acc_max=a_max)
    env.set_goal(vel_goal, pos_goal)
    env.calculate_path()
    env.show()
