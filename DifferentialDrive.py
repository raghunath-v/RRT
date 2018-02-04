from graphics import * 
import g_tools as g
from Goal import Goal
from math import atan, acos, sin, cos, sqrt, radians

class DifferentialDrive:
    def __init__(self, vel_start, pos_start, vel_max, turn_max, dt, win):
        # dynamics related
        self.vel_x = vel_start[0]
        self.vel_y = vel_start[1]
        self.vel_max = vel_max
        self.pos_x = pos_start[0]
        self.pos_y = pos_start[1]
        self.dt = dt
        self.dist_max = vel_max * dt
        
        # Specific to differential drive
        self.turn_max = turn_max
        self.beta = 1
        self.alpha = -1
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
        self.body = None
        self.velocity_arrow = None
        self.direction_arrow = None
        self.needed_direction_arrow = None
        self.body_radius = 10
        self.win = win

        self.in_rotation = False
        self.in_motion = False
        self.in_final_rotation = False
        self.goal_vel_x = 0
        self.goal_vel_y = 0
    def set_velocity(self, goal):
        if self.in_final_rotation:
            self.final_rotate()
        elif self.in_rotation:
            self.rotate()
        elif self.in_motion:
            self.move()
        else:
            if self.path_idx == self.node_count - 1:
                self.goal_vel_x = goal.vel_x
                self.goal_vel_y = goal.vel_y
                self.alpha = atan(self.goal_vel_y/self.goal_vel_x)
                self.in_final_rotation = True
                self.final_rotate()
            else: 
                # we are at a node, maybe the first one
                self.path_idx+=1
                self.next_node = self.path[self.path_idx]
                # calculate the degree needed to move
                self.beta = atan(self.vel_y/self.vel_x) # degs from pos x
                wanted_x = self.next_node.x - self.pos_x
                wanted_y = self.next_node.y - self.pos_y
                self.alpha = atan((self.next_node.y - self.pos_y)/(self.next_node.x - self.pos_x))
                if self.alpha != self.beta:
                    self.in_rotation = True
                    self.vel_x = 0
                    self.vel_y = 0
                    self.rotate()
                else:
                    self.in_motion = True
                    self.move()
        self.set_graphicals()
    
    def final_rotate(self):
        if abs(self.alpha-self.beta) <= self.turn_max*self.dt:
            self.beta = self.alpha
            self.vel_x = self.goal_vel_x
            self.vel_y = self.goal_vel_y
            self.finished = True
        elif self.alpha > self.beta:
            self.beta+=self.turn_max*self.dt
        else:
            self.beta-=self.turn_max*self.dt
    
    def rotate(self):
        if abs(self.alpha-self.beta) <= self.turn_max*self.dt:
            self.beta = self.alpha
            self.in_rotation = False
            self.in_motion = True
        elif self.alpha > self.beta:
            self.beta+=self.turn_max*self.dt
        else:
            self.beta-=self.turn_max*self.dt
    
    def move(self):
        dist = sqrt((self.pos_x - self.next_node.x)**2 + (self.pos_y - self.next_node.y)**2)
        if(dist<=self.dist_max):
            v_fin = dist / self.dist_max
            self.vel_x, self.vel_y = v_fin * cos(self.beta), v_fin * sin(self.beta)
            self.pos_x, self.pos_y = self.next_node.x, self.next_node.y
            self.in_motion = False
            self.in_rotation = False
        else:
            self.pos_x += self.dt*self.vel_max*cos(self.beta)
            self.pos_y += self.dt*self.vel_max*sin(self.beta)

    def add_path(self, path):
        # Path is a list of Nodes
        self.path = path
        self.next_node = path[0]
        self.node_count = len(path)

    def set_graphicals(self):
        draw_x = g.scale(self.pos_x)
        draw_y = g.scale(self.pos_y)
        for el in [self.body, self.velocity_arrow, self.direction_arrow, self.needed_direction_arrow]:
           if el is not None:
               el.undraw() 
        
        self.body = Circle(Point(draw_x, draw_y), self.body_radius)
        self.body.setFill('yellow')
        self.body.draw(self.win)

        if self.vel_x != 0 and self.vel_y != 0:
            self.velocity_arrow = Line(
                Point(draw_x, draw_y),
                Point(g.scale(self.pos_x +self.vel_x), g.scale(self.pos_y + self.vel_y)))
            self.velocity_arrow.setFill('black')
            self.velocity_arrow.setArrow("last")
            self.velocity_arrow.draw(self.win)

        self.direction_arrow = Line(
            Point(draw_x, draw_y),
            Point(g.scale(self.pos_x+cos(self.beta)), g.scale(self.pos_y+sin(self.beta))))
        self.direction_arrow.setFill('green')
        self.direction_arrow.setArrow("last")
        self.direction_arrow.draw(self.win)

        '''
        self.needed_direction_arrow = Line(
            Point(draw_x, draw_y),
            Point(g.scale(self.pos_x+cos(self.alpha)), g.scale(self.pos_y+sin(self.alpha))))
        self.needed_direction_arrow.setFill('red')
        self.needed_direction_arrow.setArrow("last")
        self.needed_direction_arrow.draw(self.win)
        '''