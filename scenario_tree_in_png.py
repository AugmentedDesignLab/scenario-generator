from graphviz import Source

s = Source.from_file('followvehicle.dot')
s.render('followvehicle.gv', format='png',view=True)