from graphics import Circle, Point, Line
import numpy as np
from g_tools import scale, scale_vectors
from Path import *
from Goal import Goal
from math import sqrt, cos, sin, atan
from Node import Node
from DubinCircle import DubinCircle


class DynamicPoint:
    def __init__(self, vel_start, pos_start, dt, vel_max, acc_max, win):
        # dynamics related
        self.vel_start = np.array(vel_start)
        self.pos_x = pos_start[0]
        self.pos_y = pos_start[1]
        self.dt = dt
        self.vel_max = vel_max
        self.acc_max = acc_max
        self.dist_max = vel_max * dt

        # path planning related
        self.finished = False
        self.path = None
        self.sling_path = None
        self.sling_path_calculated = None
        self.sling_vel = None
        self.sling_acc = None
        self.node_count = 0
        self.node_count_sling = 0
        self.at_node = True
        self.total_time = 0

        # graphics related
        self.body = None
        self.body_radius = 10
        self.vel_arrow = None
        self.acc_arrow = None
        self.win = win
        self.sling_path_drawables = []
        self.get_trace = False  # Change this to see the car in action or just to see the trace
        if self.get_trace is True:
            self.body_radius /= 10

        # for actions
        self.current_action = None
        self.current_acc = np.array([0, 0], dtype=float)
        self.current_vel = np.array(vel_start, dtype=float)
        self.new_action = True   #Flag if new action is received
        self.arc_length = 0
        self.precision = 0.4
        self.position_error = 0

        # for circle/arc parameters
        self.theta = 0
        self.T = 0
        self.n = 0
        self.beta = 0
        self.circle = None

    def set_velocity(self, goal):
        self.total_time += self.dt
        # check if we have reached the next node
        self.position_error = math.sqrt((self.pos_x - self.sling_path[-1][0].x)**2 + (self.pos_y - self.sling_path[-1][0].y)**2)
        #print(self.position_error, self.n)
        if self.position_error < self.precision or self.n < 0:
            # we have reached the next one, but is it
            # the goal node?

            if (len(self.sling_path) == 1):
                self.finished = True
                return
            else:
                self.current_action = self.sling_path.pop()
                self.pos_x = self.current_action[0].x
                self.pos_y = self.current_action[0].y
                vel_dummy = self.sling_vel.pop()
                #TODO Setting velocity to dirct to the next point
                if self.current_action[1] == 0:
                    direc = 0
                    if self.sling_path[-1][0].x-self.pos_x < 0:
                        direc = math.pi + math.atan((self.sling_path[-1][0].y-self.pos_y)/(self.sling_path[-1][0].x-self.pos_x))
                    else:
                        direc = math.atan((self.sling_path[-1][0].y - self.pos_y) / (self.sling_path[-1][0].x - self.pos_x))
                    vel_mag = math.sqrt(np.dot(self.current_vel, self.current_vel))
                    self.current_vel[0] = vel_mag * math.cos(direc)
                    self.current_vel[1] = vel_mag * math.sin(direc)


                self.current_acc = self.sling_acc.pop()
                self.new_action = True

        # are we going into a circle?
        if (self.current_action[1] != 0):
            if self.new_action:
                #self.set_circle_params()
                #self.circle = DubinCircle.fromArc(self.current_action[0], self.sling_path[-1][0], self.current_action[1])
                #print("Status: ",self.current_action[0], self.current_action[1], self.current_vel)
                self.circle = DubinCircle.fromVel(self.current_action[0], self.current_action[1], self.current_vel)
                self.T = self.circle.arclength(self.current_action[0], self.sling_path[-1][0]) / \
                         math.sqrt(np.dot(self.current_vel, self.current_vel))
                self.theta = self.circle.arcangle(self.current_action[0], self.sling_path[-1][0])
                #if self.theta<0:
                #    self.theta = 2*math.pi + self.theta
                self.n = self.T / self.dt
                self.beta = self.theta / self.n
                self.new_action = False
            self.move_circular()
        else:
            # we are moving straight
            if self.new_action:
                angle = atan(self.current_vel[1] / self.current_vel[0])
                acc_mag = sqrt(np.dot(self.current_acc, self.current_acc))
                vel_mag1 = sqrt(np.dot(self.current_vel, self.current_vel))
                vel_mag2 = sqrt(np.dot(self.sling_vel[-1], self.sling_vel[-1]))
                #print("vel1", vel_mag1, "vel2", vel_mag2)
                acc_sign = 1
                if(vel_mag2 < vel_mag1):
                    acc_sign = -1

                self.current_acc[0] = acc_sign * acc_mag * cos(angle)
                self.current_acc[1] = acc_sign * acc_mag * sin(angle)
                #self.current_acc = find_acc(self.current_vel, self.sling_vel[-1],
                #                           self.current_action[0].dist_to(self.sling_path[-1][0]))
                self.new_action = False
            self.move()

        #print("Velocity:", math.sqrt(np.dot(self.current_vel,self.current_vel)),
        #      "Accel:", self.current_acc)
        #self.total_time += self.dt

    def move_circular(self):
        # TODO: sign fix
        rotation_mat = np.array([[cos(self.beta), sin(self.beta)],
                                 [-sin(self.beta), cos(self.beta)]])
        position_vect = np.array([self.pos_x - self.circle.c.x, self.pos_y - self.circle.c.y])
        position_vect = position_vect @ rotation_mat
        self.pos_x = position_vect[0] + self.circle.c.x
        self.pos_y = position_vect[1] + self.circle.c.y
        #self.pos_x = (self.pos_x - self.circle.c.x) * cos(self.beta) - \
        #             (self.pos_y - self.circle.c.y) * sin(self.beta) + self.circle.c.x
        #self.pos_y = (self.pos_x - self.circle.c.x) * sin(self.beta) + \
        #             (self.pos_y - self.circle.c.y) * cos(self.beta) + self.circle.c.y
        #print("beta: ",self.beta,"radius: ",(self.pos_x-self.circle.c.x)**2 + (self.pos_y-self.circle.c.y)**2)

        self.current_vel = self.current_vel @ rotation_mat
        angle = math.atan2(-self.current_vel[0], self.current_vel[1])

        self.current_acc[0] = self.acc_max * cos(angle)
        self.current_acc[1] = self.acc_max * sin(angle)

        self.n -= 1
        #self.set_graphicals()

    def move(self):
        #print(self.current_vel)


        self.current_vel[0] = self.current_vel[0] + self.current_acc[0] * self.dt
        self.current_vel[1] = self.current_vel[1] + self.current_acc[1] * self.dt

        self.pos_x += self.dt * self.current_vel[0]
        self.pos_y += self.dt * self.current_vel[1]

        #self.set_graphicals()

    def add_path(self, path):
        # A path is a list of nodes
        self.path = path
        self.node_count = len(path)

    def add_sling_path(self, goal, obstacles=None):
        # A path is a list of nodes
        vel_series = get_velocity_series(self.path, self.vel_start, goal.vel, self.vel_max)
        acc_series = get_acceleration_series(vel_series, self.acc_max)
        self.sling_path, self.sling_vel, self.sling_acc = create_sling_path(self.path, vel_series, acc_series, obstacles=None)#TODO: set it back to obstacles
        if not self.sling_path:
            return False
        self.sling_path_calculated = self.sling_path
        #print("Path Generated : ", self.sling_path_calculated)
        self.node_count_sling = len(self.sling_path)
        self.sling_path = [el for el in reversed(self.sling_path)]
        self.sling_vel = [el for el in reversed(self.sling_vel)]
        self.sling_acc = [el for el in reversed(self.sling_acc)]
        
        '''
        if self.sling_path_calculated is not None:
            self.sling_path_drawables = [Circle(action[0].get_scaled_point(), 3) 
                for action in self.sling_path_calculated]
            for el in self.sling_path_drawables:
                el.draw(self.win)

        for el in self.sling_path:
            cir = Circle(el[0].get_scaled_point(), 3)
            cir.draw(self.win)
        '''
        #for action in self.sling_path_calculated:
        #    self.body = Circle(action[0].get_scaled_point(), self.body_radius)
        #    self.body.setFill('yellow')
        #    self.body.draw(self.win)
    
    def set_graphicals(self):
        draw_x = scale(self.pos_x)
        draw_y = scale(self.pos_y)

        if self.circle is not None and self.get_trace is False:
            dubinc = Circle(self.circle.c.get_scaled_point(), scale_vectors(self.circle.r))
            dubinc.setOutline('Green')
            dubinc.draw(self.win)

        if self.body is not None and self.get_trace is False:
            self.body.undraw()
        self.body = Circle(Point(draw_x, draw_y), self.body_radius)
        self.body.setFill('yellow')
        self.body.draw(self.win)
        if self.vel_arrow:
            self.vel_arrow.undraw()
        self.vel_arrow = Line(
            Point(draw_x, draw_y),
            Point(scale(self.pos_x + self.current_vel[0]), scale(self.pos_y + self.current_vel[1])))
        self.vel_arrow.setFill('black')
        self.vel_arrow.setArrow("last")
        self.vel_arrow.draw(self.win)
        if self.acc_arrow:
            self.acc_arrow.undraw()
        self.acc_arrow = Line(
            Point(draw_x, draw_y),
            Point(scale(self.pos_x + self.current_acc[0] * 5), scale(self.pos_y + self.current_acc[1] * 5)))
        self.acc_arrow.setFill('blue')
        self.acc_arrow.setArrow('last')
        self.acc_arrow.draw(self.win)

