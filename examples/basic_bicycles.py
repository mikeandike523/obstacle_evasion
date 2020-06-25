import sys

#add the library module to the python path
sys.path.insert(0,"..")

import matplotsim
import time
import math
from geometry import point
from utils import clamp

def simple_path_follower(bicycle,targetSpeed=10):
    if bicycle.v < targetSpeed:
        acceleration=targetSpeed/2
    else:
        acceleration=-targetSpeed/2*1.5
        
    steeringAngle = 0
    lookahead_point=bicycle.get_lookahead_point()
    left=bicycle.velocity.getLeft()
    if lookahead_point != None:
        vec_bicycle_la=lookahead_point.sub(bicycle.position)
        try:
            steeringAngle=vec_bicycle_la.angleTo(bicycle.velocity)
            if vec_bicycle_la.scalarProjOnto(left) < 0 :
                steeringAngle *= -1
        except:
            pass


   
    return (acceleration,steeringAngle)
    

simulator=matplotsim.Simulator(-100,100,-50,50,600,0.1)
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


simulator.add_bicycle(bicycle1)
simulator.add_bicycle(bicycle2)

simulator.loop()
    
