from vertexTest import Vertex
import urlTest
import json
import math

class Graph(object):

    def __init__(self):
        """ initializes a graph object """
        """ figure out what to store internally for a graph"""
        self.vertexList = []

    def __str__(self):
        """ print out the vertices and edges """
        """ you are free to print out in any output format """
        list1 = []
        for v in self.vertexList:
            list1.append(v.name)

        return str(list1)+str(self.edgeMatrix)
    def init_edges(self,num):
        self.edgeMatrix = [[(-1, -1) for x in range(num)] for y in range(num)]
    def addVertex(self, v):
        self.vertexList.append(v)

    def addEdge(self, v1, v2):
        self.edgeMatrix[v1-1][v2-1]=(self.vertexList[v1-1].caculateDis(self.vertexList[v2-1]), self.vertexList[v1-1].caculateAng(self.vertexList[v2-1]))

def dijkstra(graph,n,begin,end):
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
        mini=1000000
        for i in range(n):
            if ((dis[i]!=-1) and (dis[i]<mini)) and (not flag[i]):
                mini=dis[i]
                k=i
        if mini==1000000:
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
    exec('urlTest')

    f = open(urlTest.filename)
    f = open('COM1_2')

    jsonStr = f.readlines()[0]
    map1 = json.loads(jsonStr)

    n=0
    g = Graph()
    north=int(map1['info']['northAt'])
    for node in map1['map']:
        v = Vertex(node['nodeName'], int(node['x']), int(node['y']), int(node['nodeId']))
        n=n+1
        g.addVertex(v)
    #print(g)
    g.init_edges(n)
    for node in map1['map']:
        v = Vertex(node['nodeName'], int(node['x']), int(node['y']), int(node['nodeId']))
        string1 = node["linkTo"]
        list1 = string1.split(', ')
        for linkedNode in list1:
            g.addEdge(int(node['nodeId']), int(linkedNode))

  #  print(g)
    print("")
    print("Be sure the following vertex inputs are under " + str(n))
    begin = input("pls input the starting vertex:")
    end = input("pls input an ending vertex:")
    sh_route=dijkstra(g.edgeMatrix, n, int(begin), int(end))['route']
    print(str(sh_route))
    current=0;
    while(True):
        current_X = int(input("current X: "))
        current_Y = int(input("current Y: "))
        current_Angel=int(input("current Angel: "))

        if g.vertexList[sh_route[current]-1].x-int(current_X)<=5 and g.vertexList[sh_route[current]-1].y-int(current_Y)<=5:
            begin=sh_route[current]
            current=current+1
        ax = -(current_X - g.vertexList[sh_route[current]-1].x)
        ay = -(current_Y - g.vertexList[sh_route[current]-1].y)
        ang_cal = int(math.atan2(ax, ay) * 180 / 3.14159)

        # ang_diff=-(360-north-current_Angel)
        #
        # if (ang_diff>180):
        #     ang_diff=ang_diff-360
        # if (ang_cal-ang_diff>5 and ang_cal-ang_diff<=180) or (ang_cal-ang_diff>=-355 and ang_cal-ang_diff<-180):
        #     print("turn right "+str((ang_cal-ang_diff)%180))
        # else:
        #     if (ang_cal-ang_diff<-5 and ang_cal-ang_diff>=-180) or (ang_cal-ang_diff>180 and ang_cal-ang_diff<=355):
        #         print("turn left "+str((ang_cal-ang_diff)%180))
        #     else:
        #         print("keep straight forward")
        ang_cal = (360-north+ang_cal) % 360 # 下一个点 north向左偏的角度
        ang_diff = ang_cal - current_Angel

        if (ang_diff>5 and ang_diff<=180) or (ang_diff>=-355 and ang_diff<-180):
            print("turn right "+str(ang_diff%180))
        else:
            if (ang_diff<-5 and ang_diff>=-180) or (ang_diff>180 and ang_diff<=355):
                print("turn left "+str(ang_diff)%180)
            else:
                print("keep straight forward")

        print("the next position should be "+str(sh_route[current]))

        print(str(dijkstra(g.edgeMatrix,n,int(begin),int(end))))

#如果两点间没有连线,怎么走
