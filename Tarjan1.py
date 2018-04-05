import re
import networkx as nx
import numpy as np
import sys

# G is the name of the graph
G=nx.Graph()
# Just use maximum number of nodes and edges
Total_Nodes = 100000
Total_edges = 100000
# counter used as id in Bridge finding method
Counter=1
#prev and low are the flag of each node
prev=np.zeros(Total_Nodes+1)
low=np.zeros(Total_edges)
sys.setrecursionlimit(20000)

def min_value(value1,value2):

    if (value1<value2):
        return value1
    else:
        return value2
# this algorithm u can find in http://csengerg.github.io/2015/12/26/pre-order-travelsal-and-tarjans-algorithm.html website
def Tarjan_bridge_finding(present_vertex,parents):
    global Counter
    prev[present_vertex]=Counter
    low[present_vertex]=prev[present_vertex]
    Counter = Counter + 1

    for i in range(G.degree(present_vertex)):
        all_neighbors=list(G.neighbors(present_vertex))
        #print(w1[i])
        neighbors=all_neighbors[i]
        if(prev[neighbors]==0):
            Tarjan_bridge_finding(neighbors,present_vertex)
            low[present_vertex]=min_value(low[present_vertex],low[neighbors])
            if(low[neighbors]==prev[neighbors]):
                print("Edge",present_vertex,"to",neighbors,"Bridge")
        if(neighbors!=parents):
            low[present_vertex]=min_value(low[present_vertex],low[neighbors])


def main():
    global Total_Nodes,Total_edges
    with open("c:\python\instances\instance009.gr") as file:
        for line in file:
            val = line.strip('\n')
            # I skip everythings only use the information the vertices and edges
            if ('SECTION' in val) or ('ENDEOF' in val) or ('ENDSECTION' in val) or ('Terminals' in val) or ('END' in val) \
                    or ('EOF' in val):

                continue
            if 'Nodes' in val:
                val1 = val.strip('Nodes ')
                Total_Nodes = int(val1)
                print('Total_Nodes:  ', Total_Nodes)
                continue
            if 'Edges' in val:
                val1 = val.strip('Edges ')
                Total_edges = int(val1)
                print('Total_Edges:  ', Total_edges)
                continue
            if ('E' in val):
                val1 = val.strip('E ')
                s = re.findall(r"[-+]?\d*\.\d+|\d+", val1)

                node1=int(s[0])
                node2=int(s[1])
                G.add_edge(node1,node2)
    Tarjan_bridge_finding(node1,0)


    file.close()

main()
