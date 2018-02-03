import json
from graphics import GraphWin
from Obstacle import Obstacle
from BoundingArea import BoundingArea
from KinematicPoint import KinematicPoint
from DynamicPoint import DynamicPoint
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
        self.rrt.set_graphicals()
        #self.rrt.remove_graphicals()

    def run(self, rrt_setup):
        self.init_draw()
        self.gen_rrt(rrt_setup)
        while not self.player.finished:
            #self.player.set_auto_velocity(self.goal)
            self.player.set_velocity(self.goal)
            self.player.move()
        print("Is finshed")
        print("Position:")
        print("goal: ", self.goal.pos_x,",",self.goal.pos_y)
        print("player: ", self.player.pos_x,",",self.player.pos_y)
        print("Velocity:")
        print("goal: ", self.goal.vel_x,",",self.goal.vel_y)
        print("player: ", self.player.vel_x,",",self.player.vel_y)
        print("Time taken to reach goal (dt): ", player.total_time)
        self.win.getMouse()
        self.win.close()
    
    def init_draw(self):
         # draw everything initially
        self.bounding_area.set_graphicals()
        for obs in self.obstacles:
            obs.set_graphicals()
        self.player.set_graphicals()
        self.goal.set_graphicals()

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
    canvas_width = 800
    canvas_height = 600
    win = GraphWin("area", canvas_width, canvas_height)
    player = KinematicPoint(vel_start, pos_start, dt, v_max, win)
    goal = Goal(vel_goal, pos_goal, win)
    env = Environment(obstacles, bounding_poly, player, goal, win)
    rrt_setup = {'delta_q': 2, 'k':500, 'x_range': [-2,60], 'y_range': [-2,60]}
    env.run(rrt_setup)
