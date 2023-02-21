from collections import defaultdict
from email.policy import default
import heapq
import math as Math
from matplotlib.font_manager import list_fonts
import numpy as np
import time
from turtle import width
from matplotlib import pyplot as plt
class Node:
    def __init__(self, name):
        self.name = name
        self.edge_list = []

    def connect(self, node,wight):
        con = (self.name, node.name,wight)
        self.edge_list.append(con)

class Edge:
    def __init__(self, left, right, weight):
        self.left = left
        self.right = right
        self.weight = weight

class Graph:

    def __init__(self):
        self.verticies = {}
        self.edges = {}
    
    def add_node(self, node):
        self.verticies[node.name] = node
    
    def add_edge(self, left, right, weight):
        if left.name not in self.verticies:
            self.verticies[left.name] = left
        if right.name not in self.verticies:
            self.verticies[right.name] = right

        e = Edge(left, right, weight)

        key = (left.name, right.name)
        self.edges[key] = e

        key = (right.name, left.name)
        self.edges[key] = e

        left.connect(right,weight)
        right.connect(left,weight)

    def search(self, a, b):
        pass

    def to_aj_matrix(self):
        pass
    
    def dfscode(self,visited,path, startnode,endnode):
        if startnode.name not in path:
            # print (startnode.name)
            visited.add(startnode.name)
            path.append(startnode.name)
            for neighbour in startnode.edge_list:
                if self.verticies[neighbour[1]].name==endnode.name:
                    path.append(self.verticies[neighbour[1]].name)
                    return path
                else:
                    if neighbour[1] not in visited:
                        x = self.dfscode(visited,path, self.verticies[neighbour[1]],endnode)
                        if x:
                            return x
            path.pop()
    def dfs(self,startnode,endnode):
        visited = set()
        path=[]
        self.dfscode(visited,path, startnode,endnode)
        if path:
            return path
        else:
            return "no pathe between the nodes"
    def bfs(self ,startnode,endnode):
        visited = set()
        queue = []
        path=[]

        queue.append((startnode.name, path))
        visited.add(startnode.name)
        path.append(startnode.name)
        
        while queue:
            s, path = queue.pop(0)
            if self.verticies[s].name==endnode.name:
                return path
                break
            for i in self.verticies[s].edge_list:
                if i[1] not in visited:
                    queue.append((i[1],path+[i[1]]))
                    visited.add(i[1])
                    
        return "path not found"
    


    def dijkstra(self, s, t):
        q = []
        d = {k: float("inf") for k in self.verticies}
        p = {}
        d[s.name] = 0 
        heapq.heappush(q, (0, s.name,[]))
        while q:
            last_w, curr_v, route = heapq.heappop(q)
            route.append(curr_v)
            if t.name==curr_v:
                break
            for near in self.verticies[curr_v].edge_list: 
                n=near[1]
                n_w=near[2]
                cand_w = last_w + n_w
                if cand_w < d[n]:
                    d[n] = cand_w
                    p[n] = curr_v
                    heapq.heappush(q, (cand_w, n,route[:]))
        
        return route

    def aStar(self,s,t, file):
        long_lat=self.long_lat_reader(file)
        def huristic(lat1,lon1,lat2,lon2):
            Radius_of_earth = 6371
            dLat = degreetorad(float(lat2)-float(lat1))  
            dLon = degreetorad(float(lon2)-float(lon1)) 
            n = Math.sin(dLat/2) * Math.sin(dLat/2) + Math.cos(degreetorad(float(lat1))) * Math.cos(degreetorad(float(lat2))) * Math.sin(dLon/2) * Math.sin(dLon/2)
            c = 2 * Math.atan2(Math.sqrt(n), Math.sqrt(1-n))
            distance = Radius_of_earth * c
            return distance
        def degreetorad(degree):
            return float(degree) * (Math.pi/180)
        q = []
        d = {k: float("inf") for k in self.verticies}
        p = {}
        d[s.name] = 0 
        heapq.heappush(q, (0, 0, s.name,[]))
        while q:
            last_w, total, curr_v, route = heapq.heappop(q)
            route.append(curr_v)
            if t.name==curr_v:
                break
            for near in self.verticies[curr_v].edge_list: 
                n=near[1]
                n_w=near[2]
                total += n_w
                cand_w = total  + huristic(long_lat[near[0]][0],long_lat[near[0]][1],long_lat[t.name][0],long_lat[t.name][1])
                if cand_w < d[n]:
                    d[n] = cand_w
                    p[n] = curr_v
                    heapq.heappush(q, (cand_w, total, n,route[:]))
        return route
    def long_lat_reader(self,long1_lat):
        file1=open(long1_lat,'r')
        f1=file1.readlines()
        newlist1=[]
        for i in f1:
            
            if i[-1]=="\n":
                newlist1.append(i[:-1])
            else:
                newlist1.append(i)
        long_lat=dict()
        for i in newlist1:
            list2=i.split(" ")
            long_lat[list2[0]]=(list2[1],list2[2])
        return long_lat
    
        
    
def filereader(txtfile):  
    g = Graph()    
    file=open(txtfile,'r')
    f=file.readlines()
    newlist=[]
    for i in f:
        if i[-1]=="\n":
            newlist.append(i[:-1])
        else:
            newlist.append(i)
    for i in newlist:
        list1=i.split(" ")
        if list1[0] not in g.verticies and list1[1] not in g.verticies:
            a=Node(list1[0])
            b=Node(list1[1])
        elif list1[0] in g.verticies and list1[1] not in g.verticies:
            a=g.verticies[list1[0]]
            b=Node(list1[1])
        elif list1[0] not in g.verticies and list1[1]  in g.verticies:
            a=Node(list1[0])
            b=g.verticies[list1[1]]
        elif list1[0] in g.verticies and list1[1]  in g.verticies:
            a=g.verticies[list1[0]]
            b=g.verticies[list1[1]]
        
        g.add_edge(a, b,int(list1[2]))
    return g
# m=filereader('1x.txt').verticies["Arad"]
# n=filereader('1x.txt').verticies["Iasi"]
# print(filereader('1x.txt').dfs(m,n))

# g = filereader('1x.txt')
# ans = g.closecentralityAstar('long_lat1x.txt')
# for node in g.verticies:
#     print(node, ans[node])

  

# for iv, (k, v) in enumerate(g.verticies.items()):
#     print(v.name)
#     print(v.edge_list)

# for iv, (k, edge) in enumerate(g.edges.items()):
#     print(edge.left.name, edge.right.name, edge.weight)
    
# print(graph)
#  # Set to keep track of visited nodes.



# Driver Code


