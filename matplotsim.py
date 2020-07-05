import math
from geometry import point,segment
import matplotlib.pyplot as plt
from utils import clamp
import keyboard
#paths are global to the corordinate system even for each car
class Car:
    
    def __init__(self,x=0,y=0,yaw=math.pi/2,v=0,WB=2.9): #meters per second
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.WB=WB
        self.rear_x = self.x - ((self.WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.WB / 2) * math.sin(self.yaw))
        self.calculate_vectors()
        self.steering_behaviour=lambda car: (0,0)
        
    def update(self,acceleration=0,steeringAngle=0,timeStep=0.05):
        steeringAngle=clamp(steeringAngle,-math.pi/2,math.pi/2)
        self.x += self.v * math.cos(self.yaw) * timeStep
        self.y += self.v * math.sin(self.yaw) * timeStep
        self.yaw += self.v / self.WB * math.tan(steeringAngle) * timeStep
        self.v +=acceleration * timeStep
        self.rear_x = self.x - ((self.WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.WB / 2) * math.sin(self.yaw))
        self.calculate_vectors()
        
    def calculate_vectors(self):
        self.position=point(self.x,self.y)
        self.velocity=point(math.cos(self.yaw)*self.v,math.sin(self.yaw)*self.v)

    def plot(self):
        plt.arrow(self.x,self.y,math.cos(self.yaw)*4.5,math.sin(self.yaw)*4.5,length_includes_head=True,head_length=4.5,head_width=1.8)

    def steer(self):
       return self.steering_behaviour(self)

    def set_steering_behaviour(self,behaviour):
        self.steering_behaviour=behaviour

class Simulator():

    def __init__(self,timeStep=0.05):
        self.timeStep=timeStep
        self.ego_vehicle=None
        plt.gca().set_aspect('equal')
        self.track_boundaries=[]
  

    def loop(self):
       print("use q to quit, must focus console and hold q key breifly")
       while not keyboard.is_pressed('q'):

           #clear the axes
           plt.cla()
           #set axes extens and plot
           plt.gca().set_xlim(self.ego_vehicle.x-20,self.ego_vehicle.x+20)
           plt.gca().set_ylim(self.ego_vehicle.y-20,self.ego_vehicle.y+20)

           #plot track boundaries
           for wall in self.track_boundaries:
               plt.plot([wall.a.x,wall.b.x],[wall.a.y,wall.b.y],'b-')
       

           self.ego_vehicle.plot()
           
           #physics update
           for i in range(5):
            self.ego_vehicle.update(*self.ego_vehicle.steer(),self.timeStep)


           plt.pause(self.timeStep*2)

    def set_ego_vehicle(self,car):
        self.ego_vehicle=car

    def clear_track_boundaries(self):
        while len(self.track_boundaries) > 0:
            self.track_boundaries.pop()
        
    def add_track_boundary(self,wall):
        self.track_boundaries.append(wall)

    


#if run as a script, run a test script
if __name__=="__main__":

    simulator=Simulator(0.05)

    #build the simulator track boundaries (left and right edges of track)
    for i in range (-5,20):
        simulator.add_track_boundary(segment(point(7.62,i*10),point(7.62,i*10+10)))
        simulator.add_track_boundary(segment(point(-7.62,i*10),point(-7.62,i*10+10)))

    def basic_steering(car):
        acceleration=20-car.v
        steeringAngle=0
        return acceleration,steeringAngle
            
    car=Car()
    car.set_steering_behaviour(basic_steering)
    simulator.set_ego_vehicle(car)

    simulator.loop()
   
