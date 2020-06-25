from math import *


class point:

    def __str__(self):
        return "<{},{}>".format(self.x,self.y)
    def __init__(self,x,y):
        self.x=x
        self.y=y
    def magnitude(self):
        return sqrt(self.x*self.x+self.y*self.y)
    def normalized(self):
        length=self.magnitude()
        return point(self.x/length, self.y/length)
    def scaled(self,scale):
        return point(scale*self.x,scale*self.y)
    def sum(self,other):
        return point(self.x+other.x,self.y+other.y)
    def sub(self,other):
        return point(self.x-other.x,self.y-other.y)
    def dot(self,other):
        return self.x*other.x+self.y*other.y
    def angleOf(self):
        return atan2(self.y,self.x)
    def angleTo(self,other):
        try:
            return acos(self.dot(other)/(self.magnitude()*other.magnitude()))
        except:
            return 0
    def projOnto(self,other):
        magOther=other.magnitude()
        dotWithOther=other.dot(self)
        scalarProj=dotWithOther/magOther
        otherNorm=other.normalized()
        return otherNorm.scaled(scalarProj)
    def scalarProjOnto(self,other):
        magOther=other.magnitude()
        dotWithOther=other.dot(self)
        scalarProj=dotWithOther/magOther
        return scalarProj
    def copy(self):
        return point(self.x,self.y)
    def getLeft(self):
        return point(-self.y,self.x)
    def asTuple(self):
        return (self.x,self.y)