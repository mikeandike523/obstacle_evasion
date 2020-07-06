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
        self.collision_behaviour= lambda car: car.reset()
        self.rays=[]
     
        
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
        for ray in self.rays:
            plt.plot([ray.a.x,ray.b.x],[ray.a.y,ray.b.y],'r-')
      

    def steer(self):
       return self.steering_behaviour(self)

    def set_steering_behaviour(self,behaviour):
        self.steering_behaviour=behaviour

    def update_rays(self,walls):
            while len(self.rays) >0:
                self.rays.pop()
            for i in range (21):
                angle=-math.pi/2+math.pi*i/20+self.yaw;
                cast = self.position.sum(point(math.cos(angle)*20,math.sin(angle)*20))
                ray_len=100000000000000
                result_ray=None
                for wall in walls:
                    #for some reason both directions are needed
                    intersection=segment(self.position,cast).intersect_segment(wall)
                    if intersection is None:
                        intersection=wall.intersect_segment(segment(self.position,cast))


                    if intersection is not None:    
                        test_result_ray=segment(self.position,intersection)
                        test_ray_len=test_result_ray.b.sub(test_result_ray.a).magnitude()
                        if test_ray_len<ray_len:
                            ray_len=test_ray_len
                            result_ray=test_result_ray
                if result_ray!=None:
                    self.rays.append(result_ray)

    def collide(self):
        for ray in self.rays:
            if ray.length()<0.25:
                self.collision_behaviour(self)


    def set_collision_behaviour(self,behaviour):
        self.collision_behaviour=behaviour

    def reset(self,x=0,y=0,yaw=math.pi/2,v=0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((self.WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.WB / 2) * math.sin(self.yaw))
        self.calculate_vectors()
        while len(self.rays) > 0:
            self.rays.pop()
        

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
                plt.plot([wall.a.x,wall.b.x],[wall.a.y,wall.b.y],'k-')


            self.ego_vehicle.plot()

            #physics update
            for i in range(6):
                self.ego_vehicle.update(*self.ego_vehicle.steer(),self.timeStep)
            self.ego_vehicle.update_rays(self.track_boundaries)
            self.ego_vehicle.collide()
            plt.grid()
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
    import csv
    with open('fences.csv', 'r') as file:
        reader = csv.reader(file,delimiter='\t')
        for row in reader:
            simulator.add_track_boundary(segment(point(float(row[0]),float(row[1])),point(float(row[2]),float(row[3]))))



    def basic_steering(car):
        
        steeringAngle=0
        
        for ray in car.rays:
            try:
                rayAngle=ray.b.sub(ray.a).angleTo(car.velocity)
                if ray.b.sub(ray.a).scalarProjOnto(car.velocity.getLeft())<0:
                    rayAngle*=-1
                steeringAngle-=math.pi/40*rayAngle/abs(rayAngle)*math.exp(1-(ray.length()/8))
            except:
                #bad alignment between angles
                pass
        
        acceleration=10-car.v
        
        return acceleration,steeringAngle
            
    car=Car(50+7.62)
    car.set_steering_behaviour(basic_steering)
    car.set_collision_behaviour(lambda car: car.reset(50+7.62))
    simulator.set_ego_vehicle(car)
    simulator.loop()
   
