import graphviz

g = graphviz.Digraph('RRT Path', format = 'svg', filename='hello.gv')

g.edge('Hello', 'World')

g.view()