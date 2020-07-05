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


class segment: #also serves as a 2x2 matrix, column major

    def __init__(self,a,b):
        self.a=a.copy()
        self.b=b.copy()

    def left_multiply_point(self,pt):
        a=self.a.x
        b=self.b.x
        c=self.a.y
        d=self.b.y
        i=pt.x
        j=pt.y
        return point(a*i+b*j, c*i+d*j)
    
    def inverse(self):
        a=self.a.x
        b=self.b.x
        c=self.a.y
        d=self.b.y
        det=a*d-b*c
        if det<0.001:
            return None
        return segment(point(d,-c).scaled(1/det),point(-b,a).scaled(1/det))
    
    def intersect_segment(self,other):
        #q1=r1+t1v1
        #q2=r2+t2v2
        #r1+t1v1=r2+t2v2
        #r1-r2=[-v1 v2](t1,t2)
        #(t1,t2)=[-v1 v2]^(-1) (r1-r2)
        r1=self.a
        r2=other.a
        v1=self.b.sub(self.a)
        v2=other.b.sub(other.a)
        left_side=r1.sub(r2)
        inv=segment(v1.scaled(-1),v2).inverse()
        if inv==None:
            return None
        t=inv.left_multiply_point(left_side)
        
        t1=t.x
        t2=t.y
        if t1>0 and t1<1 and t2>0 and t2 < 1:
            return r1.sum(v1.scaled(t1))
        return None