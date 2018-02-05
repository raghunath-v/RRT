import json
from graphics import * 
import g_tools as g
from Goal import Goal
from math import atan, acos, sin, cos, sqrt, tan

class KinematicCar:
    def __init__(self, vel_start, pos_start, vel_max, length, phi_max, dt, win):
        # gives radius:  L/tan(max_phi)

        # dynamics related
        vel_x = vel_start[0]
        vel_y = vel_start[1]
        self.vel_max = vel_max
        self.pos_x = pos_start[0]
        self.pos_y = pos_start[1]
        self.dt = dt
        self.dist_max = vel_max * dt
        
        # Specific to differential drive
        self.phi_max = phi_max
        self.length = length
        self.theta = atan(vel_y/vel_x)
        # makes sense to me to set turning to 0 in refernece
        # to itself initially instead of an arbitrary 0 
        self.phi = self.theta
        self.vel_magnitude = sqrt(vel_x**2 + vel_y**2)
        # path planning related
        self.path = None
        self.node_count = 0
        self.path_idx = 0
        self.next_node = None
        self.finished = False
        self.total_time = 0
        self.in_rotation = False
        self.is_moving = False
        self.at_node = True
        self.at_new_node = False

        # graphics related
        self.body_back = None # the actual position
        self.body_front = None
        self.direction_arrow = None
        self.turning_arrow = None
        self.body_back_radius = 10
        self.body_front_radius = 5
        self.win = win

    def set_velocity(self):
        '''
            Change stuff here. Like I understand it,
            the velocity should be considered a scalar
            while theta, which is determined by phi, should
            control the direction of the vehicle. So here we
            can change the velocity scalar and the current phi
            while taking into consderation the constraints of 
            max_phi and max_velocity
        '''

    def move(self):
        self.pos_x+= self.vel_magnitude*cos(self.theta)*dt
        self.pos_y+= self.vel_magnitude*sin(self.theta)*dt
        # according to the lecture, this is the order of things
        self.theta+=(self.vel_magnitude/self.length)*tan(self.phi)*dt
        self.set_graphicals()

    def set_graphicals(self):
        draw_back_x = g.scale(self.pos_x)
        draw_back_y = g.scale(self.pos_y)
        draw_front_x = g.scale(self.pos_x + self.length*cos(self.theta))
        draw_front_y = g.scale(self.pos_y + self.length*sin(self.theta))
        
        if self.body_back is not None:
            self.body_back.undraw()
        self.body_back = Circle(Point(draw_back_x, draw_back_y), self.body_back_radius)
        self.body_back.setFill('yellow')
        self.body_back.draw(self.win)
        if self.body_front is not None:
            self.body_front.undraw()
        self.body_front = Circle(Point(draw_front_x, draw_front_y), self.body_front_radius)
        self.body_front.setFill('yellow')
        self.body_front.draw(self.win)
        if self.turning_arrow is not None:
            self.turning_arrow.undraw()
        self.turning_arrow = Line(
            Point(draw_front_x, draw_front_y),
            Point(draw_front_x+5*cos(self.theta + self.phi), draw_front_y+5*sin(self.theta + self.phi))
        )
        self.turning_arrow.setFill('green')
        self.turning_arrow.setArrow('last')
        self.turning_arrow.draw(self.win)


if __name__ == '__main__':
    with open("environments/P1.json") as json_file:
        desc = json.load(json_file)
    canvas_width = 800
    canvas_height = 800
    win = GraphWin("area", canvas_width, canvas_height)
    win.yUp()
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
    player = KinematicCar(vel_start, pos_start, v_max, vehicle_length, phi_max, dt, win)
    player.pos_x = 25
    player.pos_y = 25
    player.set_graphicals()
    for i in range(1000):
        player.move()

    win.getMouse()
    win.close()