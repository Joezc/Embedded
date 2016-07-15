import math

class Vertex:
    
    def __init__(self, name, x, y,nodeId):
        """ initializes a vertex object """
        self.name = name
        self.x = x
        self.y = y
        self.nodeId=nodeId
        
        
    def __str__(self):
        res = str(self.name) + ",ID="+self.nodeID+"(" + str(self.x) + "," + str(self.y)+")"
        return res

    
    def caculateDis(self ,b):
        ax=abs(self.x-b.x)
        ay=abs(self.y-b.y)
        dis=(ax*ax+ay*ay)**0.5
        return dis
    
    def caculateAng(self,b):
        ax=-(self.x-b.x)
        ay=-(self.y-b.y)
      #  if (ay==0):
       #     if (ax>0):
        #        ang = 90
         #   else:
          #      ang = 270
        #else:
         #   if (ax==0):
          #      if
        #ang=math.atan2(ax,ay)
        ang=int(math.atan2(ax, ay)*180/3.14159)
        #if (ang==360):
        #    ang=0
        #else:
 
        return ang 


    #Add other methods below


#Sample test of basic functionalities given
#v1 = Vertex("A", 100, 200)
#print(v1)
