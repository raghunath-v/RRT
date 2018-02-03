from graphics import Circle, Point, Line
from g_tools import scale 

class Goal:
    def __init__(self, vel, pos, win):
        self.vel_x = 0.2
        self.vel_y = 0.8
        self.pos_x = pos[0]
        self.pos_y = pos[1]
        self.win = win
    
    def set_graphicals(self):
        # draw player
        self.body = Circle(Point(scale(self.pos_x), scale(self.pos_y)), 7)
        self.body.setFill('red')
        # Note: downwards in Y is the positive direction for this graphics lib
        self.arrow = Line(Point(scale(self.pos_x), scale(self.pos_y)),
            Point(scale(self.pos_x + self.vel_x), scale(self.pos_y + self.vel_y)))
        self.arrow.setFill('black')
        self.arrow.setArrow('last')
        self.body.draw(self.win)
        self.arrow.draw(self.win)