import time
import signal
class GracefulKiller:
  kill_now = False
  def __init__(self):
    signal.signal(signal.SIGINT, self.exit_gracefully)
    signal.signal(signal.SIGTERM, self.exit_gracefully)

  def exit_gracefully(self,signum, frame):
    self.kill_now = True


killer = GracefulKiller()

from heapq import heappush, heappop
from itertools import count
from itertools import permutations
input()
n = int(input().split()[1])
e = int(input().split()[1])

from collections import defaultdict
graph = defaultdict(dict)
max_cost = 0
for x in range(e):
    s,u,v,w = input().split()
    u,v,w = map(int,(u,v,w))
    graph[u][v] = w
    graph[v][u] = w
    max_cost += w

input() # end
check = input()
if 'SECTION' not in check:input()

t = int(input().split()[1])
terminals = []
for x in range(t):
    _, ter = input().split()
    terminals.append(int(ter))

terminals = sorted(terminals,key = lambda x: (len(graph[x]),x),reverse = True)
steinertree = defaultdict(dict)

def add_edges(path):
    global steinertree
    if isinstance(path,int):return
    if len(path)<= 1:return
    for a,b in zip(path[:-1],path[1:]):
        w = graph[a][b]
        steinertree[a][b] = w
        steinertree[b][a] = w
        
def remove_edges(edges):
    global steinertree
    reduced = set((min(a,b),max(a,b)) for a,b in edges)
    for a,b in reduced:
        steinertree[a].pop(b, None)
        steinertree[b].pop(a, None)
        if len(steinertree[a]) == 0:
            steinertree.pop(a, None)
        if len(steinertree[b]) == 0:
            steinertree.pop(b, None)

def check_path(c_path):
    global steinertree
    intersection = c_path[-1]
    if c_path[0] in steinertree:
        #processed_nodes.update(closest_path)
        add_edges(closest_path)
        return
    current_dist = sum(graph[a][b] for a,b in zip(c_path[:-1],c_path[1:]))

    nodes_to_keep = []
    if intersection in terminals:
        nodes_to_keep.append(intersection)


    paths_to_intersections = []
    for v in steinertree[intersection].keys():
        prev = intersection
        current_dist += graph[v][prev]
        paths_to_intersections.append((v,prev))
        while not v in terminals and len(steinertree[v]) == 2:
            v,prev = [k for k in steinertree[v].keys() if k!= prev][0],v
            current_dist += graph[v][prev]
            paths_to_intersections.append((v,prev))
        nodes_to_keep.append(v)
    nodes_to_keep.append(c_path[0])
    
    shorter_path = shortest_connection(nodes_to_keep,current_dist)
    if shorter_path:
        remove_edges(paths_to_intersections)
        

        for p in shorter_path:
            add_edges(p)
    else:
        add_edges(closest_path)


def shortest_connection(nodes_to_connect, cut_off = None):
    if len(nodes_to_connect)!=3:return None
#     if len(nodes_to_connect)>3:
#         processed_nodes.update(closest_path)
#         add_edges(closest_path)
#         print ('Not this time', len(nodes_to_connect),nodes_to_connect)
#         return None
        
    seen_by_node = defaultdict(list)
    # fringe is heapq with 3-tuples (distance,c,node)
    # use the count c to avoid comparing nodes (may not be able to)
    c = count()
    fringe = []
    path_dict = defaultdict(dict)
    dist_dict = defaultdict(dict)
    for node in nodes_to_connect:
        path_dict[node][node] = [node]
        #dist = {}  # dictionary of final distances
        seen = {}
        push(fringe, (0, next(c), node))
        while fringe:
            (d, _, v) = pop(fringe)
            if v in dist_dict[node]:
                continue  # already searched this node.
            dist_dict[node][v] = d
            for u, cost in graph[v].items():
                vu_dist = dist_dict[node][v] + cost
                if vu_dist > cut_off:
                    continue
                if u in dist_dict[node]:
                    if vu_dist < dist_dict[node][u]:
                        seen[u] = vu_dist
                        push(fringe, (vu_dist, next(c), u))
                        path_dict[node][u] = path_dict[node][v] + [u]
                elif u not in seen or vu_dist < seen[u]:
                    seen[u] = vu_dist
                    push(fringe, (vu_dist, next(c), u))
                    path_dict[node][u] = path_dict[node][v] + [u]
        for k,v in dist_dict[node].items():
            seen_by_node[k].append(v)
    
    count_nodes = len(nodes_to_connect)
    size_permutation = count_nodes//2
    limit_close_nodes = count_nodes#count_nodes//size_permutation + size_permutation - 1
    
    #choose k i len(v) == count_nodes and closest limit_close_nodes sum < cut_off
    relevant = [k for k,v in seen_by_node.items() if len(v)==count_nodes 
                and sum(sorted(v)[:limit_close_nodes])<cut_off]
    #print ('nodes',count_nodes,'relevant', len(relevant))
    
    shorter_solution = None
    for s in range(size_permutation,size_permutation+1):
        for per in permutations(relevant,s):
            min_dist = [min(dist_dict[n][p] for p in per) for n in nodes_to_connect]
            dist_to_nodes = sum(min_dist)
            
            permutation_cutoff = cut_off - dist_to_nodes
            dist_per, paths_between_per = connect_permutation(per,permutation_cutoff)
#             dist_per = 0
#             paths_between_per = []
            if dist_per == None:continue
            
            
            if (dist_per+dist_to_nodes)<cut_off:
                cut_off = dist_per+dist_to_nodes
                best_p = [[p for p in per if d == dist_dict[n][p]][0] for d,n in zip(min_dist,nodes_to_connect)]
                shorter_solution = [path_dict[n][p] for n,p in zip(nodes_to_connect,best_p)]
                if paths_between_per:
                    shorter_solution += paths_between_per
#     relevant = sorted(relevant, key = lambda x: x[1])
#     if relevant and relevant[0][1]<cut_off:
#         return [path_dict[n][relevant[0][0]] for n in nodes_to_connect]
    return shorter_solution


def connect_permutation(perm, cut_off = None):
    if len(perm) == 1:
        return 0,[]
    if len(perm) == 2:
        return shortest_path(perm[0],perm[1],cut_off)
    sol = shortest_connection(perm, cut_off)
    if sol == None:
        return None, None
    sol_dist = 0
    for p in sol:
        if isinstance(p,int):continue
        sol_dist += sum(graph[a][b] for a,b in zip(p[:-1],p[1:]))
    return sol_dist, sol #TODO compute cost of permutation


def shortest_path(start,target,cut_off = None):
    c = count()
    fringe = []
    path = {}
    dist = {}
    
    path[start] = [start]
    #dist = {}  # dictionary of final distances
    seen = {}
    push(fringe, (0, next(c), start))
    while fringe:
        (d, _, v) = pop(fringe)
        if v in dist:
            continue  # already searched this node.
        dist[v] = d
        if v == target:continue
        for u, cost in graph[v].items():
            vu_dist = dist[v] + cost
            if vu_dist > cut_off:
                continue
            if u in dist:
                if vu_dist < dist[u]:
                    seen[u] = vu_dist
                    push(fringe, (vu_dist, next(c), u))
                    path[u] = path[v] + [u]
            elif u not in seen or vu_dist < seen[u]:
                seen[u] = vu_dist
                push(fringe, (vu_dist, next(c), u))
                path[u] = path[v] + [u]
    if target in dist:
        return dist[target],path[target]
    return None,None


best_solution = max_cost + 1
best_tree = None
for i,ttt in enumerate([terminals[0]] + terminals):
    if killer.kill_now:
        break
    #sources = terminals
    push = heappush
    pop = heappop
    dist = {}  # dictionary of final distances
    seen = {}
    # fringe is heapq with 3-tuples (distance,c,node)
    # use the count c to avoid comparing nodes (may not be able to)
    c = count()
    fringe = []
    paths = {source: [source] for source in terminals}
    start = ttt
    
    sources = [start]
    searched = set()
    for source in sources:
        seen[source] = 0
        push(fringe, (0, next(c), source))
        paths[source] = [source]
        #closest_terminal[source] = (0, [source], source) 
    while fringe:
        if killer.kill_now:
            break
        (d, _, v) = pop(fringe)
        if v in searched:
            continue  # already searched this node.
        dist[v] = d
        searched.add(v)
        for u, cost in graph[v].items():
            if killer.kill_now:
                break
            vu_dist = dist[v] + cost
    #         if cutoff is not None:
    #             if vu_dist > cutoff:
    #                 continue
            if u in dist:
                if vu_dist < dist[u]:
                    seen[u] = vu_dist
                    push(fringe, (vu_dist, next(c), u))
                    paths[u] = paths[v] + [u]
                    closest_terminal[u] = (vu_dist, paths[u], source) 
            elif u not in seen or vu_dist < seen[u]:
                seen[u] = vu_dist
                push(fringe, (vu_dist, next(c), u))
                paths[u] = paths[v] + [u]
    if killer.kill_now:
        break            
    steinertree = defaultdict(dict)
    other_terminals = sorted([_ for _ in terminals if _!=ttt],key = lambda x: (dist[x],x),reverse=False)
    closest = other_terminals[0]
    closest_path = paths[closest]
    
    add_edges(closest_path)
    
    
    for ter in other_terminals[1:]:
        if killer.kill_now:
            break
        dist = {}  # dictionary of final distances
        seen = {}
        # fringe is heapq with 3-tuples (distance,c,node)
        # use the count c to avoid comparing nodes (may not be able to)
        c = count()
        fringe = []
        paths = {ter: [ter]}
        closest_dist = 2**32
        closest_path = None
        sources = [ter]
        searched = set()

        for source in sources:
            seen[source] = 0
            push(fringe, (0, next(c), source))
            paths[source] = [source]
            #closest_terminal[source] = (0, [source], source) 
        while fringe:
            if killer.kill_now:
                break
            (d, _, v) = pop(fringe)
            if v in searched:
                continue  # already searched this node.
            dist[v] = d
            searched.add(v)
            for u, cost in graph[v].items():
                vu_dist = dist[v] + cost
                if vu_dist > closest_dist:
                    continue
                if u in dist:
                    if vu_dist < dist[u]:
                        seen[u] = vu_dist
                        push(fringe, (vu_dist, next(c), u))
                        paths[u] = paths[v] + [u]
                elif u not in seen or vu_dist < seen[u]:
                    seen[u] = vu_dist
                    push(fringe, (vu_dist, next(c), u))
                    paths[u] = paths[v] + [u]
                if u in steinertree:
                    closest_dist = vu_dist
                    closest_path = paths[u]
        if killer.kill_now:
            break
        if i>0:
            check_path(closest_path)
        else:
            add_edges(closest_path)
    #     processed_nodes.update(closest_path)
        #add_edges(closest_path)
    if killer.kill_now:
        break
    steinertree_edges = set()
    for a in steinertree.keys():
        steinertree_edges.update((min(a,b),max(a,b)) for b in steinertree[a].keys())
    tree_cost = sum(steinertree[a][b] for a,b in steinertree_edges)
    if tree_cost < best_solution:
        best_solution = tree_cost
        best_tree = steinertree_edges
    
    if killer.kill_now:
        break


print ('VALUE {}'.format(best_solution))
print ('\n'.join('{} {}'.format(a,b) for a,b in best_tree))