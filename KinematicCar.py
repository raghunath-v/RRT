from graphics import * 
import g_tools as g
from Goal import Goal
from math import atan, acos, sin, cos, sqrt

class KinematicCar:
    def __init__(self, vel_start, pos_start, vel_max, length, phi_max, dt, win):
        # give  radiuss = L/tan(max_phi)

        # dynamics related
        self.vel_x = vel_start[0]
        self.vel_y = vel_start[1]
        self.vel_max = vel_max
        self.pos_x = pos_start[0]
        self.pos_y = pos_start[1]
        self.dt = dt
        self.dist_max = vel_max * dt
        
        # Specific to differential drive
        self.phi_max = phi_max

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
        self.body_radius = 10
        self.win = win

def set_velocity(self):
    return False