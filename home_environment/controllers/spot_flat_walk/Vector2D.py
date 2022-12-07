from math import sqrt, cos, sin, isinf
from numpy import array

class Vector2D:
    def __init__(self, dx=None, dy=None, magnitude=None, alpha=None):
        self.__dr = None
        self.__magnitude = -1.0
        self.__direction = None
        self.update(dx, dy, magnitude, alpha)

    def __add__(self, other):
        vec = Vector2D(self.__dr[0], self.__dr[0])
        dr = self.getDR() + other.getDR()
        vec.update(dr[0],dr[1])
        return vec

    def __sub__(self, other):
        vec = Vector2D(self.__dr[0], self.__dr[1])
        dr = self.getDR() - other.getDR()
        vec.update(dr[0],dr[1])
        return vec

    def __mul__(self, other):
        vec = Vector2D(self.__dr[0], self.__dr[1])
        dr = self.getDR() * other
        vec.update(dr[0],dr[1])
        return vec

    def __truediv__(self, other):
        return self.__mul__(1.0 / other)

    def __update_Mag_Dir(self, magnitude, alpha):
        self.__magnitude = magnitude 
        self.__direction = array([cos(alpha),sin(alpha)])
        self.__dr=self.__magnitude*self.__direction

    def __update_DX_DY(self, dx, dy):
        if isinf(abs(dx)):
            dx = 1000000000000
            dy = 1000000000000
        self.__dr = [dx, dy]
        if type(dx) == type(list()) and type(dy) == type(list()):
            self.__dr[0] = dy[0] - dx[0]
            self.__dr[1] = dy[1] - dx[1]
        
        self.__dr=array([dx, dy])
        self.__magnitude = sqrt(dx ** 2 + dy ** 2)
        # print(self.__dr, self.__magnitude)
        if self.__magnitude > 0:
            self.__direction = (1/self.__magnitude)*self.__dr
        else:
            self.__direction = None

    def update(self, dx=None, dy=None, magnitude=None, alpha=None):
        if dx is not None and dy is not None:
            self.__update_DX_DY(dx,dy)
        elif magnitude is not None and alpha is not None:
            self.__update_Mag_Dir(magnitude,alpha)

    def getDR(self):
        return self.__dr

    # def getDX(self):
    #     return self.__dr[0]
    
    # def getDY(self):
    #     return self.__dr[1]

    def getMagnitude(self):
        return self.__magnitude
    
    def getDirection(self):
        return self.__direction

    def __repr__(self):
        return str((self.__dr[0],self.__dr[1]))