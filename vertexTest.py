import math

class Vertex:

    def __init__(self, name, x, y,nodeId):
        """ initializes a vertex object """
        self.name = name
        self.x = x
        self.y = y
        self.nodeID=nodeId


    def __str__(self):
        res = self.name + "(" + str(self.x) + "," + str(self.y) + ")"
        return res


    def caculateDis(self, b):
        ax=abs(self.x-b.x)
        ay=abs(self.y-b.y)
        dis=(ax*ax+ay*ay)**0.5
        return dis

    def caculateAng(self, b):
        ax = b.x-self.x
        ay = b.y-self.y
        ang=int(math.atan2(ax, ay)*180/math.pi)
        return ang

# Sample test of basic functionalities given
# v1 = Vertex("A", 100, 200, 1)
# print(v1)
