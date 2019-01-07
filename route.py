#!/usr/bin/env python3

# put your routing program here!

from queue import PriorityQueue
from math import radians, cos, sin, asin, sqrt
import sys

#This is the succesor function,
#Assumptions:
#1. We have taken into consideration biderectional routes
#2. If the speed on a route is 0 or not given in the road_segments.txt, we have
#   assumed the speed to be the mean speeds of all the routes
def successors(start_node):
    succ=[]
    succ_element=[]
    for i in range(len(rs)):
        if rs[i][0] == start_node:
            if (rs[i][3]=='0' ):
                speed = avgSpeed
                succ_element = [rs[i][0], rs[i][1], rs[i][2], speed, rs[i][4]]
            elif(len(rs[i])!=5):
                speed = avgSpeed
                succ_element = [rs[i][0], rs[i][1], rs[i][2], speed, rs[i][3]]
            else:
                succ_element = [rs[i][0], rs[i][1], rs[i][2], rs[i][3], rs[i][4]]
            succ.append(succ_element)
        elif rs[i][1] == start_node:
            if rs[i][3]=='0':
                speed = avgSpeed
                succ_element = [rs[i][1], rs[i][0], rs[i][2], speed, rs[i][4]]
            elif(len(rs[i])!=5):
                speed = avgSpeed
                succ_element = [rs[i][1], rs[i][0], rs[i][2], speed, rs[i][3]]
            
            else:
                succ_element = [rs[i][1], rs[i][0], rs[i][2], rs[i][3], rs[i][4]]
            succ.append(succ_element)
    return(succ)
    
#This function returns true if the node is the end node          
def is_destination(end_node):
    return end_node == end_city

#Below is the function which is used to determine the cost function(g(s)) which
#is used determine how much expensive has been the state so far. Input given is 
#the 'cost function' chosen, the distance, time, segments visited so far and the
#latest distance travelled, time taken and returns updated time, distance and
#segments visited.
def cost_Func(cost,d,s,distance_so_far,time_so_far,segment):
    
    if(cost=="segments"):
        return (segment+ 1)
    
    elif(cost=="distance"):
       # print(priority,d)
        return(distance_so_far + float(d))
    
    elif(cost=="time"):
        return (time_so_far + float(d)/float(s))
    

#Calculates the haversine distance between two cities and whose GPS points are
#given in the file city_gps.txt
def haversineDist( start_node,end_node,prev_node,d,cost):
    lat1 = 0
    lon1 = 0
    hDist=0
   
    lat2,lon2=dict[end_city]  
    for i in range(len(gps)):
        if gps[i][0] == end_node:
            lat1 = float(gps[i][1])   
            lon1 = float(gps[i][2])
            dict[end_node]=[lat1,lon1]
   
    if(lat1==0):
        lat1temp,lon1temp=dict[start_node]
        
        lat1=(lat1temp+lat2)/2
        lon1=(lon1temp+lon2)/2
        dict[end_node]=[lat1,lon1]
            
    # convert decimal degrees to radians 
    lat1,lon1,lat2,lon2 = map(radians, [lat1,lon1,lat2,lon2])
    lat = lat2 - lat1 
    lon = lon2 - lon1 
    a = sin(lat/2)**2 + cos(lat1) * cos(lat2) * sin(lon/2)**2   
    c = 2 * asin(sqrt(a)) 
    r = 3956 
    
    hDist= c*r
    if(cost=='distance'):
        return hDist
    elif(cost=='time'):
        return hDist/avgSpeed
    elif(cost=='segments'):
        return hDist/avgDist
       
#Function to implement BFS       
def solve_BFS(start_city,cost):
    closed = []
    time_taken = 0
    distance_travelled = 0
    segments=0
    route_travelled = start_city
    fringe = [(start_city, distance_travelled, time_taken, route_travelled,segments)]
    while len(fringe) > 0:
        (start_node, distance_so_far, time_so_far, route_so_far,segments) = fringe.pop()
        closed.append(start_node)
       # print(start_node)
        for (start, end, d, s, road_taken) in successors(start_node):
            if(end not in closed):
                if is_destination(end):
                    return(float(time_so_far) + float(d)/float(s), float(distance_so_far) + float(d), route_so_far + " " + end,segments+1)
                fringe.insert(0, (end, float(distance_so_far) + float(d), float(time_so_far) + float(d)/float(s), route_so_far +" " + end,segments+1))
            
    return False

#Function to implement DFS
def solve_DFS(start_city,cost):
    closed = []
    time_taken = 0
    distance_travelled = 0
    segments=0
    route_travelled = start_city
    fringe = [(start_city, distance_travelled, time_taken, route_travelled,segments)]
    while len(fringe) > 0:
        (start_node, distance_so_far, time_so_far, route_so_far,segments) = fringe.pop()
        closed.append(start_node)
     
        for (start, end, d, s, road_taken) in successors(start_node):
            if(end not in closed):
                if is_destination(end):
                    return(float(time_so_far) + float(d)/float(s), float(distance_so_far) + float(d), route_so_far + " " + end,segments+1)
                fringe.append((end, float(distance_so_far) + float(d), float(time_so_far) + float(d)/float(s), route_so_far + " " + end,segments+1))
            
    return False

#Function to implement IDS
#DFS is implement which perfroms DFS upto a certain depth(k) then backtracks 
#and checks all the nodes upto that depth      
def solve_IDS(start_city,cost):
    closed = []
    time_taken = 0
    distance_travelled = 0
    segments=0
    route_travelled = start_city
    fringe = [(start_city, distance_travelled, time_taken, route_travelled,segments)]
    k=0
    x=0
    #d=1
    while k >=0:
        while len(fringe) > 0:
            (start_node, distance_so_far, time_so_far, route_so_far,segments) = fringe.pop()
            closed.append(start_node)       
            for (start, end, d, s, road_taken) in successors(start_node):
                    if(segments<k): 
                        if is_destination(end):
                            return(float(time_so_far) + float(d)/float(s), float(distance_so_far) + float(d), route_so_far + " " + end,segments+1)
                        fringe.append((end, float(distance_so_far) + float(d), float(time_so_far) + float(d)/float(s), route_so_far +" " + end,segments+1))
                    elif x==0 :
                        closed.pop
                        x=1
            x=0          
        k+=1
        del fringe[:]
        del closed[:]
        closed= []
        fringe = [(start_city, 0, 0, start_city,0)]
     
    return False

#Function to implement IDS - 2
#Please Note: Function 'Solve_IDS' gives optimal solution upto depth 8 after 
#which it taken a lot of time to run
def solve_newIDS(start_city):
    for depth in range(1,500):
        closed = []
        time_taken = 0
        distance_travelled = 0
        route_travelled = start_city
        segments=1
        fringe = [(start_city, distance_travelled, time_taken, route_travelled, segments)]
        while len(fringe) > 0:
            (start_node, distance_so_far, time_so_far, route_so_far, segments) = fringe.pop()
            #print('route_so_far:',route_so_far,'nodes:',nodes_visited)
            closed.append(start_node)
            closed=list(set(closed))
            print(closed,segments,depth)
            if segments<depth:
               # print(start_node)
                for (start, end, d, s, road_taken) in successors(start_node):
                    if(end not in closed):
                        if is_destination(end):
                            return(float(time_so_far) + float(d)/float(s), float(distance_so_far) + float(d), route_so_far + "-" + road_taken + "-" + end,segments+1)
                        fringe.append((end, float(distance_so_far) + float(d), float(time_so_far) + float(d)/float(s), route_so_far + "-" + road_taken + "-" + end, segments+1))

    return False

#Function to implement Uniform search
#Modification of Search Algortihm 3 is implemented here which puts all states 
#in FRINGE along with cost. Cost is calculated using the 'Cost_Func' function
#based on the input given by the user
def solve_Uniform(start_city,cost):
    fringe = PriorityQueue()
    time_taken = 0
    distance_travelled = 0
    route_travelled = start_city
    visited=[]
    finalCost=0
    segments=0
    fringe.put((0,(start_city, distance_travelled, time_taken, route_travelled,segments)))
    
    while fringe:
        (priority, (start_node, distance_so_far, time_so_far, route_so_far,segments)) = fringe.get()
        if is_destination(start_node):
            return(float(time_so_far), float(distance_so_far) , route_so_far,segments)
                
        if start_node not in visited:
            visited.append(start_node)
            for (start, end, d, s, road_taken) in successors(start_node):
                finalCost = cost_Func(cost,d,s,distance_so_far,time_so_far,segments)
                if end not in visited:
                    fringe.put((finalCost, (end, float(distance_so_far) + float(d), float(time_so_far) + float(d)/float(s), route_so_far + " " + end,int(segments+1))))
    return False

#Solve function for A* 
#This function is used to calculate priority using the cost function and an 
#additional heutistic function which is calculated by checking the haversine
#distance between current node and destination node when the cost is 'distance'.
#Heuristic is admissible as the value for heuristic calculated depends on 
#the straight line distance between the node and the final destination which 
#will always underestimate the distance which is to be travelled
# ASSUMPTIONS MADE: 
#1. If speed is 0 or not given have assumed it to be mean
#2. When the cost is 'time', the havensine distance is divided by average speed
#3. When the cost is 'segments', the havensine distance is divided by average 
#   number of nodes
#4. If data is not in the city-gps.txt assumed the lat/lon to be centre btw 
#   prev point and final destination
#5. Please Note: Due to assumptions made, the result for A* is not optimal    
def solve_Astar(start_city,cost):
    fringe = PriorityQueue()
    time_taken = 0
    distance_travelled = 0
    route_travelled = start_city
    visited=[]
    finalCost=0
    segments=0
    fringe.put((0,(start_city, distance_travelled, time_taken, route_travelled,segments)))
    while fringe:
        (priority, (start_node, distance_so_far, time_so_far, route_so_far,segments)) = fringe.get()
        if start_node not in visited:
            visited.append(start_node)
            if is_destination(start_node):
                return(float(time_so_far) , float(distance_so_far) , route_so_far,segments )
            for (start, end, d, s, road_taken) in successors(start_node):
                finalCost = cost_Func(cost,d,s,distance_so_far,time_so_far,segments)
                h = haversineDist(start,end,start_node,d,cost)
                cost_func1 = finalCost+ h
                if end not in visited:
                    fringe.put((cost_func1, (end, float(distance_so_far) + float(d), float(time_so_far) + float(d)/float(s), route_so_far + " " + end,segments+1)))
  
    return False                   
    
# inputs to the function 
start_city = sys.argv[1]
end_city = sys.argv[2]
routing_algo =sys.argv[3]
cost = sys.argv[4]
    
dict={}

#read the file road-segments.txt
road_segments = open('road-segments.txt', 'r')
rs=[]
for line in road_segments:
    rows = line.split()
    rs.append([r for r in rows])
rs
road_segments.close()

#read the file city-gps.txt
city_gps = open('city-gps.txt', 'r')
gps=[]

for line in city_gps:
    rows = line.split()
    gps.append([r for r in rows])

city_gps.close()

#find the lat/lon of the start and end city
for i in range(len(gps)):
        if gps[i][0] == start_city:
            lat1 = float(gps[i][1])   
            lon1 = float(gps[i][2])
            dict[start_city]=[lat1,lon1]

        if gps[i][0] == end_city:
            lat2 = float(gps[i][1])
            lon2 = float(gps[i][2])
            dict[end_city]=[lat2,lon2]

speed=0
dist=0

#finding the average speed and average distance between nodes
for i in range(len(rs)):
    dist+=int(rs[i][2])
    if(len(rs[i])==5 and rs[1][3]!=0):
        speed+=int(rs[i][3])
avgSpeed= speed/len(rs)
avgDist = int(dist/len(rs))

#calling the different function based on the i/p
funcName = {
        "bfs" : solve_BFS,
        "uniform" : solve_Uniform,
        "dfs" : solve_DFS,
        "ids" : solve_IDS,
        "astar" : solve_Astar 
        }

func=funcName[routing_algo]

timeTaken,distance,route,segment =func(start_city,cost)

optimal=""
if(routing_algo=="dfs" or routing_algo=="ids" or routing_algo=="astar"):
    optimal="no"

elif(routing_algo=="bfs"):
    if(cost=="segments"):
        optimal="yes"
    else :
        optimal="no"
elif(routing_algo=="uniform"):
    optimal="yes"
        
print(optimal,distance,timeTaken,route)
           