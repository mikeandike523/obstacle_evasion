import sys

#add the library module to the python path
sys.path.insert(0,"..")

import matplotsim
import time
import math
from geometry import point
from utils import clamp
import random

simulator=matplotsim.Simulator(-100,100,-50,50,600,0.1)
def simple_path_follower(bicycle,targetSpeed=15):
   
        
    lookahead_point=bicycle.get_lookahead_point()
    steeringAngle=math.pi/8
    left=bicycle.velocity.getLeft()
    if lookahead_point != None:
        simulator.plot(lookahead_point)
     
        vec_bicycle_la=lookahead_point.sub(bicycle.position)
        try:
            steeringAngle=vec_bicycle_la.angleTo(bicycle.velocity)
            if vec_bicycle_la.scalarProjOnto(left) < 0 :
                steeringAngle *= -1
        except:
            pass
    targetSpeed*=(1-steeringAngle/(2*math.pi))       
    if bicycle.v < targetSpeed:
        acceleration=targetSpeed/2
    else:
        acceleration=-targetSpeed/2*1.5
    if bicycle.target != None:        
        if bicycle.position.sub(bicycle.target).magnitude() < 15:
            bicycle.set_target(point(random.randint(-45,45),random.randint(-25,25)))
    return (acceleration,steeringAngle)
    

bicycle1=matplotsim.Bicycle(37,0,math.pi/2,10)
for i in range(33):
    angle = 2*math.pi/32*i
    bicycle1.add_to_path(point(math.cos(angle)*35,math.sin(angle)*35))


bicycle1.set_steering_behaviour(simple_path_follower)


bicycle2=matplotsim.Bicycle(22,0,math.pi/2,10)
for i in range(33):
    angle = 2*math.pi/32*i
    bicycle2.add_to_path(point(math.cos(angle)*20,math.sin(angle)*20))


bicycle2.set_steering_behaviour(simple_path_follower)




bicycle3=matplotsim.Bicycle(-70,0,0,0)
#target is to get to 70,0
def simple_planner(bicycle,opponents):
    bicycle.clear_path()
    vec_bicycle_target=bicycle.target.sub(bicycle.position)
    vec_bicycle_targetn=vec_bicycle_target.normalized()
    distance=vec_bicycle_target.magnitude()
    try:
        for i in range(math.floor(distance/0.1)):
            bicycle.add_to_path(bicycle.position.sum(vec_bicycle_targetn.scaled(i*0.1)))
        for i in range(len(bicycle.path)):
            path_point=bicycle.path[i].copy()
            for opponent in opponents:
                    path_point=path_point.sum(opponent.position.sub(path_point).normalized().scaled(-1*30/opponent.position.sub(path_point).magnitude()))
                    bicycle.path[i]=path_point
    except:
        pass


bicycle3.set_update_path_behaviour(simple_planner)
bicycle3.set_steering_behaviour(simple_path_follower)
bicycle3.set_target(point(random.randint(-45,45),random.randint(-25,25)))   
simulator.add_bicycle(bicycle1)
simulator.add_bicycle(bicycle2)
simulator.add_bicycle(bicycle3)
simulator.loop()