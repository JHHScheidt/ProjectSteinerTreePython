import networkx as nx
file = open('./Data/public/instance039.gr').read().splitlines()

n = int(file[1].split()[1])
e = int(file[2].split()[1])


G = nx.Graph()
for x in range(3, 3+e):
    s,u,v,w = file[x].split()
    G.add_edge(int(u),int(v),weight = w)


from collections import deque
processing = deque()
processing.append(1)
elements = len(G.nodes()) + 1 
parents = [None] * elements
checked = [False] * elements
parents[1] = 'root'


count = 0
val = [-1] * elements
low = [-1] * elements
while processing:
    current = processing.popleft()
    if checked[current]:continue
    children = G[current].keys()
    processing.extendleft(children)
    
    for c in children:
        if not checked[c]:
            parents[c] = current
    checked[current] = True
    val[current] = count
    low[current] = count
    count += 1


## Update low values
done = False
while not done:
    done = True
    for node in G.nodes():
        parent = parents[node]
        children = G[node].keys()
        current_val = val[node]
        children_vals = [(val[c], low[c])[val[c]>current_val] for c in children if c!=parent] + [low[node]]
        min_low = min(children_vals)
        if low[node] > min_low:
            low[node] = min_low
            done = False

# find bridges
bridges = []
for node in G.nodes():
    node_val = val[node]
    for c in G[node].keys():
        if low[c]>node_val:
            bridges.append((node,c))
print (len(bridges))