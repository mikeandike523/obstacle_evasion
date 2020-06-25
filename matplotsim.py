import math
from geometry import point
import pygame
clock = pygame.time.Clock()

#paths are global to the corordinate system even for each bicycle
class Bicycle:
    def __init__(self,x,y,yaw,v,WB=2.9):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.WB=2.9
        self.rear_x = self.x - ((self.WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.WB / 2) * math.sin(self.yaw))
        self.path=[]
        self.calculate_vectors()
        self.steering_behaviour=lambda bicycle: (0,0)
        self.update_path_behaviour=lambda bicycle, opponents: None
        self.target=None
    def update(self,acceleration=0,steeringAngle=0,timeStep=0.1):
        self.x += self.v * math.cos(self.yaw) * timeStep
        self.y += self.v * math.sin(self.yaw) * timeStep
        self.yaw += self.v / self.WB * math.tan(steeringAngle) * timeStep
        self.v +=acceleration * timeStep
        self.rear_x = self.x - ((self.WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.WB / 2) * math.sin(self.yaw))
        self.calculate_vectors()
        
    def clear_path(self):
        while len(self.path) > 0:
            self.path.pop()
            
    def add_to_path(self,pt):
        self.path.append(pt.copy())
        
    def calculate_vectors(self):
        self.position=point(self.x,self.y)
        self.velocity=point(math.cos(self.yaw)*self.v,math.sin(self.yaw)*self.v)
        
    def get_lookahead_point(self,lookahead_distance=8,distance_resolution=0.05):
        min_dist=100000000
        nearest=None
        for i in range(len(self.path)-1):
           
                this=self.path[i]
                next=self.path[i+1]
                dirvec=next.sub(this)
                dirvecn=dirvec.normalized()
                for j in range(math.floor(dirvec.magnitude()/distance_resolution)):
                    try:
                        newvec=this.sum(dirvecn.scaled(j*distance_resolution))
                        distvec=newvec.sub(self.position)
                        dist=distvec.magnitude()
                        if dist<min_dist and dist < 2*lookahead_distance and dist >=lookahead_distance and distvec.scalarProjOnto(self.velocity) >0.01:
                            min_dist=dist
                            nearest=newvec
                    except:
                        pass
        return nearest
        
    def set_steering_behaviour(self,behaviour):
        self.steering_behaviour=behaviour
    def steer(self):
        return self.steering_behaviour(self)
    def set_update_path_behaviour(self,behaviour):
        self.update_path_behaviour=behaviour
    def update_path(self,opponents):
        self.update_path_behaviour(self,opponents)
    def set_target(self,target):
        self.target=target.copy()

class Simulator():

    def __init__(self,minX=-50,maxX=50,minY=-50,maxY=50,xPixels=600,timeStep=0.1):
        self.minX=minX
        self.maxX=maxX
        self.minY=minY
        self.maxY=maxY
        self.timeStep=timeStep
        self.aspect=(maxX-minX)/(maxY-minY)
        self.xPixels=xPixels
        self.yPixels=math.floor(xPixels/self.aspect)
        pygame.init()
        self.screen = pygame.display.set_mode((self.xPixels, self.yPixels))
        self.bicycles=[]
        self.path=[]
        
    def getScreenCoords(self,worldCoords):
        scaleFactorX=self.xPixels/(self.maxX-self.minX)
        screenX=scaleFactorX*(worldCoords.x-self.minX)
        scaleFactorY=self.yPixels/(self.maxY-self.minY)
        screenY=self.yPixels-scaleFactorY*(worldCoords.y-self.minY)
        return point(screenX,screenY)
    
    def getWorldCoords(self,screenCoords):
        scaleFactorX=self.xPixels/(self.maxX-self.minX)
        worldX=self.minX+screenCoords.x/scaleFactorX
        scaleFactorY=self.yPixels/(self.maxY-self.minY)
        worldY=self.minY+(self.yPixels-screenCoords.y)
        return point(worldX,worldY)

        
    def loop(self):
        done = False
        while not done:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
            
            self.screen.fill((255,255,255))
            for bicycle in self.bicycles:
                screenCoords=self.getScreenCoords(bicycle.position)
                pygame.draw.ellipse(self.screen,(255,0,0),pygame.Rect(screenCoords.x-5,screenCoords.y-7,15,15))
                if bicycle.target != None:
                    screenCoords2=self.getScreenCoords(bicycle.target)
                    pygame.draw.ellipse(self.screen,(255,0,255),pygame.Rect(screenCoords2.x-5,screenCoords2.y-7,10,10))
                velocScreenCoords=self.getScreenCoords(bicycle.position.sum(bicycle.velocity))
                pygame.draw.line(self.screen,(0,0,255),screenCoords.asTuple(),velocScreenCoords.asTuple(),2)
                if len(bicycle.path)>=2:
                    for i in range(len(bicycle.path)-1):
                        
                        #point 1 screen coords
                        pt1sc=self.getScreenCoords(bicycle.path[i])
                        pt2sc=self.getScreenCoords(bicycle.path[i+1])
                        pygame.draw.line(self.screen,(0,255,0),pt1sc.asTuple(),pt2sc.asTuple(),1)
                        
                    
            
            for bicycle in self.bicycles:
                a,d=bicycle.steer()
                opponents=[]
                for test_bicycle in self.bicycles:
                    if test_bicycle is not bicycle:
                        opponents.append(test_bicycle)
                bicycle.update_path(opponents)
                bicycle.update(a,d)
                
            pygame.display.flip()
            clock.tick(1/self.timeStep)

    def add_bicycle(self,bicycle):
        self.bicycles.append(bicycle)
    
    def plot(self,pt):
        screenCoords=self.getScreenCoords(pt)
        pygame.draw.ellipse(self.screen,(0,0,0),(screenCoords.x-2,screenCoords.y-2,5,5),0)
    

   
