domain = '/root/git/JEDAI/modules/Keva Planks/domain.pddl'
problem = '/root/git/JEDAI/modules/Keva Planks/Single Pi/problem.pddl'


from state_space2 import StateSpace

ss = StateSpace(domain,problem)
print(ss.bfs(3))

# new_data = ''

# goals = "\n GOALS HERE \n"
# with open(problem,'r') as file:
#     filedata = file.read()

#     new_data = "".join([filedata.split("(:init")[0],"(:goal",filedata.split("(:goal")[1]])
#     print(new_data)
