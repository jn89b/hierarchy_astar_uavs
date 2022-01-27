import heapq

complex_graph={
    'S':{'A':[5,7],'B':[9,3],'D':[6,6]},
    'A':{'B':[3,3],'G1':[9,0]},
    'B':{'A':[2,7],'C':[1,4]},
    'C':{'S':[6,4],'G2':[5,0],'F':[7,6]},
    'D':{'S':[1,5],'C':[2,4],'E':[2,5]},
    'E':{'G3':[7,0]},
    'F':{'G3':[8,0]},
    'G1':{},
    'G2':{},
    'G3':{}
}



def astar(graph,start_node,end_node):
    # astar: F=G+H, we name F as f_distance, G as g_distance, 
    # H as heuristic
    f_distance={node:float('inf') for node in graph}
    f_distance[start_node]=0
    
    g_distance={node:float('inf') for node in graph}
    g_distance[start_node]=0
    
    came_from={node:None for node in graph}
    came_from[start_node]=start_node
    
    queue=[(0,start_node)]
    while queue:
        current_f_distance,current_node=heapq.heappop(queue)

        if current_node == end_node:
            print('found the end_node')
            return f_distance, came_from
        for next_node,weights in graph[current_node].items():
            temp_g_distance=g_distance[current_node]+weights[0]
            if temp_g_distance<g_distance[next_node]:
                g_distance[next_node]=temp_g_distance
                heuristic=weights[1]
                f_distance[next_node]=temp_g_distance+heuristic
                came_from[next_node]=current_node
                
                heapq.heappush(queue,(f_distance[next_node],next_node))
    return f_distance, came_from

vals = astar(complex_graph,'A','F')