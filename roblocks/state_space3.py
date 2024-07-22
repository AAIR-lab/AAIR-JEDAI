import pddlpy
import tempfile, shutil, os
import random

class StateSpace():


  def __init__(self,domain,problem,running_ff=False):

    temp_dir = tempfile.gettempdir()

    domain_temp_path = os.path.join(temp_dir, 'domain_copy.pddl')
    shutil.copy2(domain, domain_temp_path)
    self.searchAndReplace(domain_temp_path,"=","equals")

    self.domprob = pddlpy.DomainProblem(domain_temp_path, problem)

    problem_temp_path = os.path.join(temp_dir, 'problem_copy.pddl')
    with open(problem, 'r') as file :
      filedata = file.read()

      final_problem_pddl = filedata
    
      if not running_ff:      
        parts = filedata.split(":init")
        k = []
        for obj in self.domprob.problem.objects.keys():
          k.append("".join(['(equals ',obj,' ',obj,')']))
        equality_str = ("\n".join(k))

        final_problem_pddl = (("\n").join([parts[0],":init",equality_str,parts[1]]))


    with open(problem_temp_path, 'w+') as file:
      file.write(final_problem_pddl)


    self.domprob = pddlpy.DomainProblem(domain_temp_path, problem_temp_path)

    os.remove(domain_temp_path)
    os.remove(problem_temp_path)


    self.state = frozenset([self.stringifyAtom(i) for i in self.domprob.problem.initialstate])

    
    self.grounded_operators = []
    self.grounded_operators_dict = {}

    for operator in self.domprob.operators():
      for grounded_operator in self.domprob.ground_operator(operator):
        self.grounded_operators.append(grounded_operator)
        str_value = grounded_operator.operator_name +" " + " ".join(grounded_operator.variable_list.values())
        self.grounded_operators_dict[str_value] = grounded_operator

  def implementPlan(self,plan):
    visited_states = set()
    visited_states.add(self.state)

    if plan == '':
      print("Empty plan in ss ")
      return self.state

    for action in plan.split(","):
      print(action)
      print(self.state)
      print("=========================")
      grounded_operator = self.grounded_operators_dict.get(action,None)

      if not grounded_operator:
        msg = "".join(["Invalid action ",action," in plan"])
        raise Exception(msg)
      
      if not self.isActionApplicable(grounded_operator,self.state):
        msg = "".join(["Cant implement action ",action," in plan"])
        raise Exception(msg)      

      next_state = self.applyAction(grounded_operator,self.state)
      
      if next_state in visited_states:
        print("REPEAT!!!!")
        return False
      else:
        visited_states.add(next_state)

      self.state = next_state

    return self.state

  def stringifyAtom(self,atom):
    return tuple(str(atom).replace(",","").replace(")","").replace("(","").replace("'","").split(" "))


  def isActionApplicable(self,grounded_operator,state):
    return (grounded_operator.precondition_pos.issubset(state) and
            grounded_operator.precondition_neg.isdisjoint(state))

  def applyAction(self,grounded_operator,state):
    return (state.difference(grounded_operator.effect_neg)).union(grounded_operator.effect_pos)




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

      # print(state," ",level," ",plan)
      # print("==============================")

      longest_plan_length = max(longest_plan_length,level)

      random.shuffle(self.grounded_operators)

      for grounded_operator in self.grounded_operators:
        if self.isActionApplicable(grounded_operator,state):

          next_state = self.applyAction(grounded_operator,state)

          if(next_state not in visited_states):
            visited_states.add(next_state)

            next_plan = plan.copy()
            next_plan.append((grounded_operator.operator_name,grounded_operator.variable_list))
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

      for grounded_operator in self.grounded_operators:
        if self.isActionApplicable(grounded_operator,state):

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

    return None



  def searchAndReplace(self,fileName,find,replace):
      with open(fileName, 'r') as file :
        filedata = file.read()

        #  Replace the target string
      filedata = filedata.replace(find, replace)

        # Write the file out again
      with open(fileName, 'w') as file:
        file.write(filedata)
