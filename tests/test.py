import sys

#add the library module to the python path
sys.path.insert(0,"..")

import matplotsim
import time
import math
import keyboard


graphics=matplotsim.Graphics(-50,50,-50,50)
bicycle1=matplotsim.Bicycle(0,0,0,0)
bicycle2=matplotsim.Bicycle(0,-5,0,0)
graphics.add_bicycle(bicycle1)
graphics.add_bicycle(bicycle2)

for tick in range(100):

    #press q to quit
    if not keyboard.is_pressed('q'):
        graphics.draw()
        bicycle1.update(5,0)
        bicycle2.update(5,math.pi/10)
        time.sleep(0.1)


