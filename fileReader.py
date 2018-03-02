import networkx as nx
file = open('./Data/public/instance001.gr').read().splitlines()

n = int(file[1].split()[1])
e = int(file[2].split()[1])


G = nx.Graph()
for x in range(3, 3+e):
    s,u,v,w = file[x].split()
    G.add_edge(u,v,weight = w)

t = int(file[6+e].split()[1])

terminals = []
for x in range(7+e, 7+e+t):
    _, ter = file[x].split()
    terminals.append(ter)