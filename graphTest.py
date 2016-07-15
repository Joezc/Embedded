from vertexTest import Vertex
import json

class Graph(object):

    def __init__(self):
        """ initializes a graph object """
        """ figure out what to store internally for a graph"""
        self.vertexList = []
        self.edgeMatrix = [[(-1,-1) for x in range(100)] for y in range(100)]
        #under c: int matrix[100][100]; maxtrix[1][2]=len;
    def __str__(self):
        """ print out the vertices and edges """
        """ you are free to print out in any output format """
        list1 = []
        for v in self.vertexList:
            list1.append(v.name)
        
        return str(list1)+str(self.edgeMatrix)
            
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
    dis[begin]=0
    for i in range(n):
        dis[i]=(graph[k][i])[0]
        
    for j in range(n-1):
        mini=1000000
        for i in range(n):
            if ((dis[i]!=-1) and (dis[i]<mini)) and (not flag[i]):
                mini=dis[i]
                k=i
        if k==0:
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
    while pre[now]!=begin:
        dic['route'].append(now+1)
        now = pre[now]
    dic['route'].append(now+1)
    dic['route'].reverse()
    return dic
    
if __name__ == "__main__":
    f = open('a.txt')
    jsonStr = f.readlines()[0]
    map1 = json.loads(jsonStr)

    n=0
    g = Graph()
    for node in map1['map']:
        v = Vertex(node['nodeName'], int(node['x']), int(node['y']), int(node['nodeId']))
        n=n+1
        g.addVertex(v)
    #print(g)    
    for node in map1['map']:
        v = Vertex(node['nodeName'], int(node['x']), int(node['y']), int(node['nodeId']))
        string1 = node["linkTo"]
        list1 = string1.split(', ')
        for linkedNode in list1:
            g.addEdge(int(node['nodeId']), int(linkedNode))

    #print(g)

    print(str(dijkstra(g.edgeMatrix,n,1,10)))

