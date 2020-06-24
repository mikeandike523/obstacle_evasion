import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib
from geometry import point

class Bicycle:
    def __init__(self,x,y,yaw,v,WB=2.9):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.WB=2.9
        self.rear_x = self.x - ((self.WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.WB / 2) * math.sin(self.yaw))
    def update(self,acceleration=0,steeringAngle=0,timeStep=0.1):
        self.x += self.v * math.cos(self.yaw) * timeStep
        self.y += self.v * math.sin(self.yaw) * timeStep
        self.yaw += self.v / self.WB * math.tan(steeringAngle) * timeStep
        self.v +=acceleration * timeStep
        self.rear_x = self.x - ((self.WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.WB / 2) * math.sin(self.yaw))
class Graphics():

    def __init__(self,minX=-50,maxX=50,minY=-50,maxY=50):
        self.minX=minX
        self.maxX=maxX
        self.minY=minY
        self.maxY=maxY
        self.bicycles=[]
    

    def draw(self):
        plt.cla()
        plt.gca().set_xlim(self.minX,self.maxX)
        plt.gca().set_ylim(self.minY,self.maxY)
        plt.grid()
 

        for bicycle in self.bicycles:
            plt.arrow(bicycle.x, bicycle.y, 4.0 * math.cos(bicycle.yaw), 4.0 * math.sin(bicycle.yaw),
                  fc="r", ec="k", head_width=2.0, head_length=2.0)
            plt.plot(bicycle.x, bicycle.y,"ok")
        plt.pause(0.001)
    def add_bicycle(self,bicycle):
        self.bicycles.append(bicycle)

    def close():
        plt.close()



   