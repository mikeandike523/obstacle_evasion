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
    steeringAngle=0
    left=bicycle.velocity.getLeft()
    isFinding=False
    if lookahead_point is None:
        vec_bicycle_mouse=simulator.mouse().sub(bicycle.position)
        steeringAngle=math.pi/6
        try:
            if vec_bicycle_mouse.scalarProjOnto(left)<0:
                steeringAngle*=-1
        except:
            pass
        isFinding=True
    else:    
        simulator.plot(lookahead_point)
        vec_bicycle_la=lookahead_point.sub(bicycle.position)
        try:
     
            steeringAngle=vec_bicycle_la.angleTo(bicycle.velocity)
            if vec_bicycle_la.scalarProjOnto(left) < 0 :
                steeringAngle *= -1
        except:
            steeringAngle=math.pi/6
   
        
    
    #targetSpeed*=(1-steeringAngle/(2*math.pi))       
    if bicycle.v < targetSpeed:
        acceleration=targetSpeed/2
    else:
        acceleration=-targetSpeed/2*1.5
    
   # if isFinding:
      #  acceleration=5

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
    target=simulator.mouse()
    bicycle.clear_path()
    rope_point=bicycle.position.copy()
    bicycle.add_to_path(rope_point)
    for i in range (100):
        next_point=rope_point.sum(target.sub(rope_point).normalized().scaled(1))
        for opponent in opponents:
            next_point=next_point.sum(opponent.position.sub(next_point).normalized().scaled(-20/(opponent.position.sub(next_point).magnitude()*len(opponents))))
        angle=next_point.sub(rope_point).angleOf()
        next_point=rope_point.sum(point(math.cos(angle)*1,math.sin(angle)*1))
        bicycle.add_to_path(next_point)
        rope_point=next_point
    


bicycle3.set_update_path_behaviour(simple_planner)
bicycle3.set_steering_behaviour(simple_path_follower)
simulator.add_bicycle(bicycle1)
simulator.add_bicycle(bicycle2)
simulator.add_bicycle(bicycle3)
simulator.loop()