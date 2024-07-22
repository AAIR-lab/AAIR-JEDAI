import Explainer.explainer_server as srv
import random

def get_objects(x):
  objects = {}
  for obj in x['objects']:
    obj_type = obj['type']
    obj_name = obj['name']

    if obj_type not in objects:
      objects[obj_type] = []
    objects[obj_type].append(obj_name)
  return objects


def create_action(objects,actions,length):
  
  length = 1
  action_string = ''
  for i in range(length):
    action = random.sample(actions,1)[0]
    name = action['name']

    action_string += name
    action_string += ' '

    for param in action['params']:
      action_string += random.sample(objects[param['type']],1)[0]
      action_string += ' '
  
    action_string = action_string[:-1]

  return action_string



def run(semantics,problem_info,length):
  objects = get_objects(problem_info)
  actions = problem_info['actions']

  random_plan = ''

  for i in range(length):
    while(True):
        action = create_action(objects,actions,1)
        candidate = ''
        
        if(random_plan==''):
          candidate = action
        else:
          candidate = random_plan + ',' + action
        
        explanation_map, exec_plan = srv.call_server(candidate,semantics)

        # if (not explanation_map['failed']) or (explanation_map['err_code'] == 'GOAL ERROR'):
        if (not explanation_map['failed']) or (explanation_map['err_code'] != 'BAD ACTION'):          
          random_plan = candidate
          break

  srv.call_server(random_plan,semantics)
  return random_plan  



