from graphics import *
import numpy as np
from Obstacle import Obstacle

#Globals:
WIN = GraphWin("P1", 1000, 1000)

class KinematicPoint:
    def __init__(self, vel_start, vel_goal, pos_start, pos_goal):
        self.vel_x = vel_start[0]
        self.vel_y = vel_start[1]
        self.vel_goal_x = vel_goal[0]
        self.vel_goal_y = vel_goal[1]
        self.pos_x = pos_start[0]
        self.pos_y = pos_start[1]
        self.pos_goal_x = pos_goal[0]
        self.pos_goal_y = pos_goal[1]
    

    def show(self):
        # draw player
        point = Circle(Point(self.pos_x, self.pos_y), 5)
        point.setFill('green')
        # Note: downwards in Y is the positive direction for this graphics lib
        arrow = Line(Point(self.pos_x, self.pos_y),
            Point(self.pos_x + 10*self.vel_x, self.pos_y - 10*self.vel_y))
        arrow.setFill('red')
        point.draw(WIN)
        arrow.draw(WIN)
        # draw goal
        point = Circle(Point(self.pos_goal_x, self.pos_goal_y), 5)
        point.setFill('red')
        # Note: downwards in Y is the positive direction for this graphics lib
        arrow = Line(Point(self.pos_goal_x, self.pos_goal_y),
            Point(self.pos_goal_x + 10*self.vel_goal_x, self.pos_goal_y - 10*self.vel_goal_y))
        arrow.setFill('green')
        point.draw(WIN)
        arrow.draw(WIN)

    def play(self):
        self.show()
        '''
        points = [[20,20],[100,20],[100,100],[20,100]]
        obs = Obstacle(points)
        obs_g = obs.get_graphical()
        obs_g.setFill('blue')
        obs_g.draw(WIN)
        '''
        WIN.getMouse()
        WIN.close()

p = KinematicPoint([0.9, -0.2], [0.5, -0.5], [-15*10,2*10], [10*10, 15*10])
p.play()