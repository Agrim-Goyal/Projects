import numpy as np
#defining classes
#agent refers to the object moving from start to stop coordinates with given constraints
class Agent:
    def __init__(self, start, stop, constraints):
        self.start = start
        self.stop = stop
        self.constraints = constraints

#node refers to a coordinate in the middle of the path which has coord coordinates and object4 arrives at time from the prev node
class Node:
    def __init__(self, prev, coord,time):
        self.prev = prev
        self.coord = coord
#g=cost to reach till this node from the start, h=heuristic function which predicts cost from node to endpoint, f=g+h
        self.g = 0
        self.h = 0
        self.f = 0
        self.time=time

#constraint_node is the node in the constraint tree which has solution and branches from parent node
class constraint_Node():
    def __init__(self, solution,parent):
        self.parent=parent
        constraint= []
        self.constraint = constraint
        self.solution = solution
        self.cost= 0    

#implementation of A_star
def A_star(maze, agent,constraints):
    start = Node(None, agent.start,0)
    open_list = [start]
#note that I have removed the closed list usually used in A_star as the problem statement mentions that objects cannot wait so in some cases, the best path might be one in which some coordinate needs to be revisited
    while open_list:
#calculating the node with least f value       
        current = open_list[0]
        current_index = 0
        for i in range(1, len(open_list)):
            if open_list[i].f < current.f:
                current = open_list[i]
                current_index = i
        open_list.pop(current_index)
#printing result if node achieved is the endpoint
        if current.coord == agent.stop:
            path = []
            while current is not None:
                path.append(current.coord)
                current = current.prev
            return path[::-1]
#creating children nodes 
        for move in [[0, 1], [1, 0], [0, -1], [-1, 0],[1,-1],[1,1],[-1,-1],[-1,1]]:
            pos = [current.coord[0] + move[0], current.coord[1] + move[1]]
            if pos[0] < 0 or pos[0] >= maze_len or pos[1] < 0 or pos[1] >= maze_len:
                continue
#checking for obstacles
            if maze[pos[0]][pos[1]] != 0:
                continue
            new_node = Node(current, pos,current.time+1)
            new_node.g = current.g + 1
            new_node.h = abs(new_node.coord[0] - agent.stop[0]) + abs(new_node.coord[1] - agent.stop[1])
            new_node.f = new_node.g + new_node.h
#constraint check
            if [new_node.coord,new_node.time] in constraints:
                continue
#we remove the checking mechanism for a coordinate already existing in the open list because in some case this might lead to an ideal solution not being taken 
            open_list.append(new_node)

#low level search for solutions for each agent depending on currently existing constraints
def low_level(agents,maze,constraints):
    a=[]
    for i in range(len(agents)):
        a.append(A_star(maze,agents[i],constraints[i]))
    return a

#cost calculation for solution
def SIC(solution):
    cost=0
    for i in solution:
        cost+=len(i)-1
    return cost

#validity checker to ensure no collision
def validity(solution):
    for i in range(len(solution)):
        for j in range(i+1,len(solution)):
            for k in range(min(len(solution[i]),len(solution[j]))):
                if solution[i][k]==solution[j][k]:
                    return [i,j,solution[i][k],k]
    return 0

#high level CBS implementation
def CBS(maze,agents):
    root=constraint_Node([],None)
    p=[]
    for i in range(len(agents)):
        p.append([])
    root.solution=low_level(agents,maze,p)
    root.cost=SIC(root.solution)
#stores the nodes at the bottom of the tree, we don't store tree because we can get preceding nodes by backtracking
    open=[]
    open.append(root)
    while open is not None:
        x=0
        min=float("inf")
#choosing node with lowest cost solution
        for i in range(len(open)):
            if open[i].cost<min:
                min=open[i].cost
                x=i
        n=open.pop(x)
#if valis, we return
        flag=validity(n.solution)
        if flag==0: return n.solution
        conflict=flag
#we create child nodes with added constraint
        for i in range(2):
            next_node=constraint_Node(n.solution,n)
            next_node.constraint=[conflict[i],conflict[2],conflict[3]]
            con=next_node
            constraints=[]
            for j in range(len(agents)):
                constraints.append([])
#backtracking to get all constraints
            while con.constraint!=[]:
                constraint=con.constraint
                constraints[constraint[0]].append([constraint[1],constraint[2]])
                con=con.parent
#adding new node with solution to open list
            next_node.solution=low_level(agents,maze,constraints)
            open.append(next_node)
        
#taking user input for maze dimension, obstacles, agents and implementing CBS
maze_len = int(input("Enter square maze dimension: "))
maze=np.zeros((maze_len,maze_len),dtype="int")
n = int(input("Enter number of agents: "))
agents = []

n_obs=int(input("Enter number of obstacles: "))
for i in range(n_obs):
    y=int(input("Enter obstacle y coordinate: "))
    x=int(input("Enter obstacle x coordinate: "))
    maze[y][x]=1
for i in range(n):
    start_y = int(input("Enter starting y coordinate: "))  
    start_x = int(input("Enter starting x coordinate: "))
    stop_y = int(input("Enter stopping y coordinate: "))
    stop_x = int(input("Enter stopping x coordinate: "))
    agents.append(Agent([start_y,start_x], [stop_y,stop_x], []))
for i in CBS(maze, agents):
        print(i)