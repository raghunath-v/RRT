from graphics import *
import numpy as np
import g_tools as g

class Goal:
    def __init__(self, vel, pos, win):
        self.vel_x = vel[0]
        self.vel_y = vel[1]
        self.pos_x = pos[0]
        self.pos_y = pos[1]
        self.win = win
        self.set_graphicals()
    
    def set_graphicals(self):
        # draw player
        body = Circle(Point(g.scale(self.pos_x), g.scale(self.pos_y)), 7)
        body.setFill('red')
        # Note: downwards in Y is the positive direction for this graphics lib
        arrow = Line(Point(g.scale(self.pos_x), g.scale(self.pos_y)),
            Point(g.scale(self.pos_x + self.vel_x), g.scale(self.pos_y + self.vel_y)))
        arrow.setFill('black')
        arrow.setArrow('last')
        body.draw(self.win)
        arrow.draw(self.win)