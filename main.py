import sys
from Node import Node
from queue import PriorityQueue
import pandas as pd 
import timeit

data1 = pd.read_csv("driving.csv", index_col = 0)
data2 = pd.read_csv("straightline.csv", index_col=0)

def algorithm(initial_str, goal_str,algorithm):
    count = 0       #used to count the number of expanded nodes
    initial_node = Node(initial_str, None, 0, data2[initial_str][goal_str], algorithm) #from Node.py
    frontier = PriorityQueue()  
    frontier.put(initial_node)      #Put the first state in priority queue 

    reached = {initial_str: initial_node} #Create a dictionary with initial node in it
    while not frontier.empty():     
        nd=frontier.get()      #Pop the first node
        if nd.STATE == goal_str:
            return (nd, count) #returns the first node and the count 
        else:
            count+=1    #Increment by 1 if we expand the node
            for child in expand(goal_str,nd,algorithm):
                s = child.STATE
                #If we have not reached the state yet or the pathcost for current child is less than the one where we've already been
                if s not in reached or child.PATHCOST<reached[s].PATHCOST: 
                    reached[s] = child      #Add the entry s in child 
                    frontier.put(child)     #Put child in priority queue 
    return "failure"


def expand(goal_str, parent_node, algorithm): #Takes goal,parent node & algorithm to expand children nodes
    p = parent_node    
    neighbors = []          #stored in the form of a list 
    for s in data1.columns:         #traverse through driving distance data
        if data1[p.STATE][s]>0:             #if data>0 
            neighbors.append(Node(s, p, p.PATHCOST + data1[p.STATE][s], data2[s][goal_str], algorithm)) #append and calculate path cost plus heuristics
    return neighbors        


def main():
    print("Kumar","Aditi","A20522087 solution:")
    numberOfArgumentsPassedFromCommandLine = len(sys.argv)
    print("Number of arguments passed (including your script name):", numberOfArgumentsPassedFromCommandLine)
    if numberOfArgumentsPassedFromCommandLine >3:
        print("Error: Not enough or too many input arguments")
        exit()
    else:
        firstArgument = sys.argv[0]         
        print("\nScript name:", sys.argv[0])        
        initial = sys.argv[1]           #initial
        print("\nInitial State", initial)   
        goal = sys.argv[2]              #goal 
        print("\nGoal State", goal)
    

    if algorithm == "failure":          #if no path is found
        print("Solution path: FAILURE: NO PATH FOUND")
        print("Number of states on a path: 0")      
        print("Path cost: 0")
    else:
        result = algorithm(initial, goal, 'GBFS')

        nd=result[0]   
        path = [nd.STATE]    
        while  nd.PARENT != None :      
            nd = nd.PARENT              #calculates in reverse order from goal to parent until parent is none 
            path.append(nd.STATE)       #used to calculate solution path 
        
        path_str = path[len(path)-1]        #reverse solution path to show from initial to goal 
        for i in range(len(path)-1):
            path_str += ", "
            path_str += path[len(path)-2-i]
        
        timeStart = timeit.default_timer()      #time taken for gbfs 

        result = algorithm(initial, goal, 'GBFS')

        timeEnd = timeit.default_timer()
        elapsedTimeInSec = timeEnd - timeStart
           
        
        print("Greedy Best first search:")      #GBFS SOLUTION  
        print("Solution path:",path_str)
        print("Number of states on a path: ",len(path))
        print("Number of expanded nodes:",result[1])
        print("Path cost:",result[0].PATHCOST)
        print("Execution time: ",elapsedTimeInSec)
        print("\n")

        timeStart = timeit.default_timer()          #time taken for Astar 
        result = algorithm(initial, goal, 'ASTAR')      
        timeEnd = timeit.default_timer()
        elapsedTimeInSec = timeEnd - timeStart
        
        nd=result[0]
        path = [nd.STATE]
        while  nd.PARENT != None :
            nd = nd.PARENT
            path.append(nd.STATE)

        path_str = path[len(path)-1]
        for i in range(len(path)-1):
            path_str += ", "
            path_str += path[len(path)-2-i]
        
        print("A* Search:")                  #A* solution
        print("Solution path:",path_str)
        print("Number of states on a path: ",len(path))
        print("Number of expanded nodes:",result[1])
        print("Path cost:",result[0].PATHCOST)
        print("Execution time:",elapsedTimeInSec)
        
if __name__ == '__main__':
    main()
