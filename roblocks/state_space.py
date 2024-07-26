# import path
# import sys
# directory = path.path(__file__).abspath()
# sys.path.append(directory.parent.parent)


from .pddl_parser.pddl_parser.PDDL import  PDDL_Parser
from .pddl_parser.pddl_parser.planner import  Planner
from . import global_vars

import tempfile, shutil, os
import random
import heapq

class StateSpace():


  def __init__(self,domain,problem,running_ff=False):

    temp_dir = tempfile.gettempdir()

    domain_temp_path = os.path.join(temp_dir, 'domain_copy.pddl')
    shutil.copy2(domain, domain_temp_path)
    self.searchAndReplace(domain_temp_path,"=","equals")

    self.parser = PDDL_Parser()
    self.parser.parse_domain(domain_temp_path)
    self.parser.parse_problem(problem)

    problem_temp_path = os.path.join(temp_dir, 'problem_copy.pddl')
    with open(problem, 'r') as file :
      filedata = file.read()

      final_problem_pddl = filedata
    
      if not running_ff:      
        parts = filedata.split(":init")
        k = []
        objects = [o for l in self.parser.objects.values() for o in l]
        for obj in objects:
          k.append("".join(['(equals ',obj,' ',obj,')']))
        equality_str = ("\n".join(k))

        final_problem_pddl = (("\n").join([parts[0],":init",equality_str,parts[1]]))


    with open(problem_temp_path, 'w+') as file:
      file.write(final_problem_pddl)


    self.parser = PDDL_Parser()
    self.parser.parse_domain(domain_temp_path)
    self.parser.parse_problem(problem)

    os.remove(domain_temp_path)
    os.remove(problem_temp_path)


    self.state = self.parser.state

    ground_actions = []

    
    self.grounded_operators = []
    self.grounded_operators_dict = {}


    for action in self.parser.actions:
        for grounded_operator in action.groundify(self.parser.objects, self.parser.types):
            if("starting_point"==grounded_operator.parameters[-1]):
              str_value = grounded_operator.name +" " + " ".join(grounded_operator.parameters)
              print(f"skipping {str_value}")
              continue
            self.grounded_operators.append(grounded_operator)
            str_value = grounded_operator.name +" " + " ".join(grounded_operator.parameters)
            self.grounded_operators_dict[str_value.lower()] = grounded_operator







  def implementPlan(self,plan,action_costs,user,dom):
    visited_states = set()
    visited_states.add(self.state)
    
    if plan == '':
      print("Empty plan in ss ")
      return self.state, [self.state]
    
    state_sequence = []
    # print(global_vars.action_costs)
    
    for action in plan.split(","):
      action = action.lower()
      # print(self.state)
      state_sequence.append(self.state)
      grounded_operator = self.grounded_operators_dict.get(action,None)
      print(action)
      
      print("=========================")

      if not grounded_operator:
        msg = "".join(["Invalid action ",action," in plan"])
        raise Exception(msg)
      
      if not self.isActionApplicable(grounded_operator,self.state):
        msg = "".join(["Cant implement action ",action," in plan"])
        print(msg)
        action_costs[user][dom][grounded_operator.name] = max(action_costs[user][dom][grounded_operator.name]-1,0)
        # action_costs[dom][grounded_operator.name] = max(action_costs[dom][grounded_operator.name]-1,0)

        return self.state, state_sequence
        # raise Exception(msg)      

      action_costs[user][dom][grounded_operator.name] += 1    
      # print("inside implement plan")
      # print("dom = ",dom)
      # print("grounded_operator.name = ",grounded_operator.name)
          
      # action_costs[dom][grounded_operator.name] += 1          
      next_state = self.applyAction(grounded_operator,self.state)
      
      
      # if next_state in visited_states:
      #   print("REPEAT!!!!")
      #   return False
      # else:
        
      visited_states.add(next_state)

      self.state = next_state

    
    print(global_vars.action_costs)
    state_sequence.append(self.state)
    return self.state, state_sequence
  

  def stringifyAtom(self,atom):
    return tuple(str(atom).replace(",","").replace(")","").replace("(","").replace("'","").split(" "))


  def isActionApplicable(self,grounded_operator,state):
    return (grounded_operator.positive_preconditions.issubset(state) and
            grounded_operator.negative_preconditions.isdisjoint(state))

  def applyAction(self,grounded_operator,state):
    return (state.difference(grounded_operator.del_effects)).union(grounded_operator.add_effects)




  def dfs(self,plan_length):
    self.visited_states = set()
    self.visited_states.add(self.state)
    return self.run_dfs(self.state,plan_length,[])
  
  def run_dfs(self,curr_state, plan_length, curr_plan):

    if plan_length == 0:
      print(len(self.visited_states))
      return (True,curr_state,curr_plan)

    for grounded_operator in self.grounded_operators:

      if self.isActionApplicable(grounded_operator,curr_state):
        next_state = self.applyAction(grounded_operator,curr_state)

        if next_state not in self.visited_states:
          self.visited_states.add(next_state)

          next_plan = curr_plan.copy()
          next_plan.append(grounded_operator.operator_name +" " + " ".join(grounded_operator.variable_list.values()))

          next_dfs = self.run_dfs(next_state,plan_length-1,next_plan)
          
          if next_dfs[0] == True:
            return next_dfs
        # else:
          # print("REPEAT")
    
    return (False, curr_state, curr_plan)





  def getLongestPlan(self):
    visited_states = set(self.state)
    fringe = list()
    fringe.append((self.state,[],0))

    longest_plan_length = 0

    while fringe:
      state,plan,level = fringe.pop(0)

      print(state," ",level," ")
      # print("==============================")

      longest_plan_length = max(longest_plan_length,level)

      random.shuffle(self.grounded_operators)

      for grounded_operator in self.grounded_operators:
        if self.isActionApplicable(grounded_operator,state):

          next_state = self.applyAction(grounded_operator,state)



          if(next_state not in visited_states):
            visited_states.add(next_state)

            next_plan = plan.copy()
            next_plan.append((grounded_operator.name,grounded_operator.parameters))
            next_level = level+1

            fringe.append((next_state,next_plan,next_level))

          
    # print(len(visited_states))
    return longest_plan_length




  def bfs(self,plan_length):
    
    visited_states = set()
    visited_states.add(self.state)
    fringe = list()
    fringe.append((self.state,[],0))

    while fringe:
      state,plan,level = fringe.pop(0)

      random.shuffle(self.grounded_operators)

      num_branches = 0
      for grounded_operator in self.grounded_operators:
        if self.isActionApplicable(grounded_operator,state):
          num_branches += 1
          # print(state," ",plan," ",level)

          next_state = self.applyAction(grounded_operator,state)

          if(next_state not in visited_states):
            
            visited_states.add(next_state)

            next_plan = plan.copy()
            next_plan.append(grounded_operator)
            next_level = level+1

            if(next_level == plan_length):
              return (next_state,next_plan,next_level)

            fringe.append((next_state,next_plan,next_level))

      print("level = ",level,"num_branches = ",num_branches)
    return None


  def greedy_best_first(self, plan_length,action_costs,user,dom):
    
    visited_states = set()
    visited_states.add(self.state)
    fringe = list()
    num_actions_learned = 0
    
    
    count = 0
    #prioirty, count, state, plan, depth
    fringe.append((0,count,self.state,[],0))
    heapq.heapify(fringe)
    
    while fringe:
      priority, count, state, plan , level = heapq.heappop(fringe)
      random.shuffle(self.grounded_operators)
      if priority==0:
        num_actions_learned += 1
        
      for grounded_operator in self.grounded_operators:
        if self.isActionApplicable(grounded_operator,state):

          next_state = self.applyAction(grounded_operator,state)

          if(next_state not in visited_states):
            
            visited_states.add(next_state)

            next_plan = plan.copy()
            next_plan.append(grounded_operator)
            next_level = level+1
            count = count+1
            next_priority = priority+action_costs[user][dom][grounded_operator.name]

            # next_priority = priority+action_costs[dom][grounded_operator.name]

            if(next_level == plan_length or action_costs[user][dom][grounded_operator.name]==0):
              
              return (next_state,next_plan,next_level)

            heapq.heappush(fringe, (next_priority, count, next_state,next_plan,next_level))

    return None
      



  def searchAndReplace(self,fileName,find,replace):
      with open(fileName, 'r') as file :
        filedata = file.read()

        #  Replace the target string
      filedata = filedata.replace(find, replace)

        # Write the file out again
      with open(fileName, 'w') as file:
        file.write(filedata)