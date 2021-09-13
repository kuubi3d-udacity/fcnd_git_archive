import networkx as nx
G = nx.DiGraph() # a directed graph
G.add_edge('a', 'b')
parent = G.predecessors('b')
print("parent", parent, G.predecessors('b'))
print (G)


G.pred['b']

"""  
 {'a': {}} In [1]: import networkx as nx
 In [2]: G = nx.DiGraph() # a directed graph
 In [3]: G.add_edge('a', 'b')
 In [4]: G.predecessors('b')
 Out[4]: ['a']
 In [5]: G.pred['b']
 Out[5]: {'a': {}}
"""

