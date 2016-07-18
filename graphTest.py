from vertexTest import Vertex
import urlTest
import json
import math
import sys
import IOwithSpeak
MAXN = 100000000

class Graph(object):

    def __init__(self):
        """ initializes a graph object """
        """ figure out what to store internally for a graph"""
        self.vertexList = []
        self.nodeNum = 0

    def __str__(self):
        """ print out the vertices and edges """
        list1 = []
        for v in self.vertexList:
            list1.append(v.name)

        return str(list1)+str(self.edgeMatrix)

    def init_edges(self):
        self.edgeMatrix = [[(-1, -1) for x in range(self.nodeNum)] for y in range(self.nodeNum)]

    def addVertex(self, v):
        self.vertexList.append(v)
        self.nodeNum = self.nodeNum+1

    def addEdge(self, v1, v2):
        self.edgeMatrix[v1-1][v2-1]=(self.vertexList[v1-1].caculateDis(self.vertexList[v2-1]), self.vertexList[v1-1].caculateAng(self.vertexList[v2-1]))

    def dijkstra(self, begin, end):
        graph = self.edgeMatrix
        n = self.nodeNum

        begin = begin-1
        end = end-1

        dis=[0]*n
        flag=[False]*n
        pre=[0]*n

        flag[begin]=True
        k=begin

        for i in range(n):
            dis[i]=(graph[k][i])[0]
        dis[begin] = 0
        #print(str(dis))
        for j in range(n-1):
            mini=MAXN
            for i in range(n):
                if ((dis[i]!=-1) and (dis[i]<mini)) and (not flag[i]):
                    mini=dis[i]
                    k=i
            if mini==MAXN:
                return
            flag[k]=True
            for i in range(n):
                if (graph[k][i][0]!=-1) and ((dis[i]>dis[k]+graph[k][i][0]) or (dis[i]==-1)):
                    dis[i]=dis[k]+graph[k][i][0]
                    pre[i]=k


        dic = {}
        dic['value'] = dis[end]
        dic['route']=[]
        now = end
        while pre[now]!=0:
            dic['route'].append(now+1)
            now = pre[now]
        dic['route'].append(now+1)
        dic['route'].append(begin+1)
        dic['route'].reverse()
        return dic

if __name__ == "__main__":
    # load map information
    while True:
        jsonStr = urlTest.loadMap()
        if (jsonStr != 'fail'):
            break
    map1 = json.loads(jsonStr)

    g = Graph()
    north=int(map1['info']['northAt'])
    for node in map1['map']:
        v = Vertex(node['nodeName'], int(node['x']), int(node['y']), int(node['nodeId']))
        g.addVertex(v)

    g.init_edges()
    for node in map1['map']:
        v = Vertex(node['nodeName'], int(node['x']), int(node['y']), int(node['nodeId']))
        string1 = node["linkTo"]
        list1 = string1.split(', ')
        for linkedNode in list1:
            g.addEdge(int(node['nodeId']), int(linkedNode))

    print("")
    IOwithSpeak.outputWithVoice("Be sure the following vertexs are under %d" % g.nodeNum)
    begin = IOwithSpeak.inputWithVoice("please input the starting vertex:")
    end = IOwithSpeak.inputWithVoice("please input an ending vertex:")
    sh_route=g.dijkstra(int(begin), int(end))['route']
    print(str(sh_route))

    current=0
    while(True):
        current_X = int(IOwithSpeak.inputWithVoice("current X: "))
        current_Y = int(IOwithSpeak.inputWithVoice("current Y: "))
        current_Angel=int(IOwithSpeak.inputWithVoice("current Angel: "))

        if abs(g.vertexList[sh_route[current]-1].x-int(current_X))<=25 and abs(g.vertexList[sh_route[current]-1].y-int(current_Y))<=25:
            begin=sh_route[current]
            if (int(begin) == int(end)):
                IOwithSpeak.outputWithVoice("get the destination")
                sys.exit()
            current=current+1

        ax = -(current_X - g.vertexList[sh_route[current]-1].x)
        ay = -(current_Y - g.vertexList[sh_route[current]-1].y)
        ang_cal = int(math.atan2(ax, ay) * 180 / 3.14159)
        # print('ang_cal1= %d' % ang_cal)
        ang_cal = (360-north+ang_cal) % 360
        ang_diff = ang_cal - current_Angel

        # print('ang_cal= %d' % ang_cal)
        # print('ang_diff= %d' % ang_diff)

        ang_precison = 5
        if (ang_diff >= -360 and ang_diff <= -360 + ang_precison):
            IOwithSpeak.outputWithVoice("keep straight forward")
        elif (ang_diff > -360 + ang_precison and ang_diff <= -180 - ang_precison):
            IOwithSpeak.outputWithVoice("turn right "+str(ang_diff+360))
        elif (ang_diff > -180 - ang_precison and ang_diff <= -180 + ang_precison):
            IOwithSpeak.outputWithVoice("turn about")
        elif (ang_diff > -180 + ang_precison and ang_diff <= -ang_precison):
            IOwithSpeak.outputWithVoice("turn left "+str(-ang_diff))
        elif (ang_cal > -ang_precison and ang_diff <= ang_precison):
            IOwithSpeak.outputWithVoice("keep straight forward")
        elif (ang_diff > ang_precison and ang_diff <= 180 - ang_precison):
            IOwithSpeak.outputWithVoice("turn right "+str(ang_diff))
        elif (ang_diff > 180 -ang_precison and ang_diff <= 180 + ang_precison):
            IOwithSpeak.outputWithVoice("turn about")
        elif (ang_diff > 180 + ang_precison and ang_diff < 360 - ang_precison):
            IOwithSpeak.outputWithVoice("turn left "+str(360-ang_diff))
        elif (ang_diff >= 360 - ang_precison and ang_diff <= 360):
            IOwithSpeak.outputWithVoice("keep straight forward")

        IOwithSpeak.outputWithVoice("the next position should be "+str(sh_route[current]))

        nowDis  = math.sqrt(float( (g.vertexList[sh_route[current]-1].x-int(current_X) )**2 ) +float( (g.vertexList[sh_route[current]-1].y-int(current_Y) )**2) )
        nowDis = int(nowDis)
        
        IOwithSpeak.outputWithVoice("the next position should be "+str(sh_route[current]))
        IOwithSpeak.outputWithVoice("the distance "+str(nowDis))

        print(str(g.dijkstra(int(begin),int(end))))
