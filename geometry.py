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

class segment:

    def __init__(self,a,b):
        self.a=a
        self.b=b
    
    def intersect_segment(self,other):
        #q1=r1+t1v1
        #q2=r2+t2v2
        #r1+t1v1=r2+t2v2
        #r1-r2=[-v1 v2](t1,t2)
        #(t1,t2)=[-v1 v2]^(-1) (r1-r2)
        r1=self.a
        r2=other.a
        v1=self.b.sub(self.a).normalized()
        len1=v1.magnitude()
        v2=other.b.sub(other.a).normalized()
        len2=v2.magnitude()
        left_side=r1.sub(r2)
        determinant=(-v1.x*v2.y)-(v2.x*(-v1.y))
        if determinant < 0.001:
            return None
        inv1=point(v2.y,v1.y).scaled(1/determinant)
        inv2=point(-v2.x,-v1.x).scaled*(1/determinant)
        t1=left_side.x*inv1.x+left_side.y*inv2.x
        t2=left_side.x*inv1.x+left_side.y*inv2.x
        if t1>0 and t1<len1 and t2>0 and t2 < len2:
            return r1.sum(v1.scaled(t1)).copy() #.copy() just in case (probs not needed)