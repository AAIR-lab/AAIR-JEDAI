import os
import requests

# CAFEWORLD_DOMAIN_STR = """0
# (define (domain cafeWorld)
#     (:requirements :typing :strips)
#     (:types
#         can
#         manipulator
#         robot
#         location - object
#         table counter_loc - location
#     )

#     (:predicates
#         (empty ?gripper - manipulator)
#         (ingripper ?obj - can ?gripper - manipulator)
#         (at ?loc - location ?r - robot)
#         (order ?obj - can ?loc - location)
#     )


#     (:action move_to_counter
#         :parameters(?fromLoc - location  ?r - robot ?toLoc - counter_loc)
#         :precondition(and
#             (at ?fromLoc ?r)
#             (not (at ?toLoc ?r))
#         )

#         :effect(and
#             (at ?toLoc ?r)
#             (not (at ?fromLoc ?r))
#         )
#     )

#     (:action move_to_table
#         :parameters(?fromLoc - location  ?r - robot ?toLoc - table)
#         :precondition(and
#             (at ?fromLoc ?r)
#             (not (at ?toLoc ?r))
#         )

#         :effect(and
#             (at ?toLoc ?r)
#             (not (at ?fromLoc ?r))
#         )
#     )






#     (:action grasp
#         :parameters(?g - manipulator ?loc - location ?obj - can ?r - robot)
#         :precondition(and
#             (empty ?g)
#             (order ?obj ?loc)
#             (at ?loc ?r)
#         )
#         :effect(and
#             (not (empty ?g))
#             (ingripper ?obj ?g)
#             (not (order ?obj ?loc))

#         )
#     )
#     (:action put
#         :parameters(?g - manipulator ?loc - location ?obj - can ?r - robot)
#         :precondition(and
#             (ingripper ?obj ?g)
#             (at ?loc ?r)
#         )
#         :effect(and
#             (not (ingripper ?obj ?g))
#             (empty ?g)
#             (order ?obj ?loc)
#         )
#     )
# )
# """

# KEVA_DOMAIN_STR = """
# (define (domain KevaDeterministic)
#   (:requirements :strips :typing)

#   (:types robot plank gripper)

#   (:predicates
#     (onTable ?p - plank)
#     (onSinglePlank ?p1 - plank ?p2 - plank)
#     (onDoublePlank ?p1 - plank ?p2 - plank ?p3 - plank)
#     (clearPlank ?p - plank)
#     (empty_gripper ?gripper - gripper )
#     (inGripper ?gripper - gripper ?p - plank)
#     (horizontal ?p - plank)
#     (vertical ?p - plank)
#     (sideways ?p - plank)
#     (placed ?p - plank)
#     (can_reach ?gripper - gripper ?plank - plank)
#   ) 

#   (:action pickUp_plank_from_table
#     :parameters (?gripper - gripper ?plank - plank ?robot - robot)
#     :precondition (and
#       (empty_gripper ?gripper)
#       (clearPlank ?plank)
#       (onTable ?plank)
#       (can_reach ?gripper ?plank))
#     :effect (and
#       (not (empty_gripper ?gripper))
#       (inGripper  ?gripper ?plank)
#       (not (clearPlank ?plank))
#       (not (onTable ?plank))
#     )
#   ) 

#   (:action putDown_plank_vertical_onTable
#     :parameters (?gripper - gripper ?p - plank ?rob - robot)
#     :precondition (and
#       (inGripper ?gripper ?p)
#       (not (placed ?p)))
#     :effect (and
#       (empty_gripper ?gripper)
#       (onTable ?p)
#       (clearPlank ?p)
#       (vertical ?p)
#       (not (inGripper ?gripper ?p))
#       (placed ?p))
#   )

#   (:action putDown_plank_sideways_onTable
#     :parameters (?gripper - gripper ?p - plank ?rob - robot)
#     :precondition (and
#       (inGripper ?gripper ?p)
#       (not (placed ?p))
#     )
#     :effect (and
#       (empty_gripper ?gripper)
#       (onTable ?p)
#       (clearPlank ?p)
#       (sideways ?p)
#       (not (inGripper ?gripper ?p))
#       (placed ?p))
#   )

#   (:action putDown_plank_horizontal_onPlank
#     :parameters (?gripper - gripper ?p1 - plank ?p2 - plank ?rob - robot)
#     :precondition (and
#       (inGripper  ?gripper ?p1)
#       (not (= ?p1 ?p2))
#       (not (placed ?p1))
#       (placed ?p2))
#     :effect (and
#       (empty_gripper ?gripper)
#       (onSinglePlank ?p1 ?p2)
#       (not (clearPlank ?p2))
#       (horizontal ?p1)
#       (not (inGripper  ?gripper ?p1))
#       (clearPlank ?p1)
#       (placed ?p1))
#   )

#   (:action putDown_plank_vertical_onPlank
#     :parameters (?gripper - gripper ?p1 - plank ?p2 - plank ?rob - robot)
#     :precondition (and
#       (inGripper  ?gripper ?p1)
#       (not (= ?p1 ?p2))
#       (not (placed ?p1))
#       (placed ?p2))
#     :effect (and
#       (empty_gripper ?gripper)
#       (onSinglePlank ?p1 ?p2)
#       (not (clearPlank ?p2))
#       (vertical ?p1)
#       (not (inGripper  ?gripper ?p1))
#       (clearPlank ?p1)
#       (placed ?p1))
#   )

#   (:action putDown_plank_sideways_onPlank
#     :parameters (?gripper - gripper ?p1 - plank ?p2 - plank ?rob - robot)
#     :precondition (and
#       (inGripper ?gripper ?p1)
#       (not (= ?p1 ?p2))
#       (not (placed ?p1))
#       (placed ?p2))
#     :effect (and
#       (onSinglePlank ?p1 ?p2)
#       (not (clearPlank ?p2))
#       (sideways ?p1)
#       (not (inGripper ?gripper ?p1))
#       (clearPlank ?p1)
#       (placed ?p1))
#   )

#   (:action putDown_plank_horizontal_onDoublePlank_both_horizontal
#     :parameters (?gripper - gripper ?p1 - plank ?p2 - plank ?p3 - plank ?rob - robot)
#     :precondition (and
      
#       (inGripper ?gripper ?p1)
#       (not (= ?p1 ?p2))
#       (not (= ?p1 ?p3))
#       (not (= ?p2 ?p3))
#       (not (placed ?p1))
#       (placed ?p2)
#       (placed ?p3)
#       (horizontal ?p2)
#       (horizontal ?p3))
#     :effect (and (empty_gripper ?gripper) 
#       (not (inGripper ?gripper ?p1))
#       (onDoublePlank ?p1 ?p2 ?p3)
#       (horizontal ?p1)
#       (not (clearPlank ?p2))
#       (not (clearPlank ?p3))
#       (clearPlank ?p1)
#       (placed ?p1))
#   )

#   (:action putDown_plank_sideways_onDoublePlank_both_horizontal
#     :parameters (?gripper - gripper ?p1 - plank ?p2 - plank ?p3 - plank ?rob - robot)
#     :precondition (and
      
#       (inGripper ?gripper ?p1)
#       (not (= ?p1 ?p2))
#       (not (= ?p1 ?p3))
#       (not (= ?p2 ?p3))
#       (not (placed ?p1))
#       (placed ?p2)
#       (placed ?p3)
#       (horizontal ?p2)
#       (horizontal ?p3))
#     :effect (and (empty_gripper ?gripper) 
#       (not (inGripper ?gripper ?p1))
#       (onDoublePlank ?p1 ?p2 ?p3)
#       (sideways ?p1)
#       (not (clearPlank ?p2))
#       (not (clearPlank ?p3))
#       (clearPlank ?p1)
#       (placed ?p1))
#   )

#   (:action putDown_plank_horizontal_onDoublePlank_both_vertical
#     :parameters (?gripper - gripper ?p1 - plank ?p2 - plank ?p3 - plank ?rob - robot)
#     :precondition (and
      
#       (inGripper ?gripper ?p1)
#       (not (= ?p1 ?p2))
#       (not (= ?p1 ?p3))
#       (not (= ?p2 ?p3))
#       (not (placed ?p1))
#       (placed ?p2)
#       (placed ?p3)
#       (vertical ?p2)
#       (vertical ?p3))
#     :effect (and (empty_gripper ?gripper) 
#       (not (inGripper ?gripper ?p1))
#       (onDoublePlank ?p1 ?p2 ?p3)
#       (horizontal ?p1)
#       (not (clearPlank ?p2))
#       (not (clearPlank ?p3))
#       (clearPlank ?p1)
#       (placed ?p1))
#   )

#   (:action putDown_plank_sideways_onDoublePlank_both_vertical
#     :parameters (?gripper - gripper ?p1 - plank ?p2 - plank ?p3 - plank ?rob - robot)
#     :precondition (and
      
#       (inGripper ?gripper ?p1)
#       (not (= ?p1 ?p2))
#       (not (= ?p1 ?p3))
#       (not (= ?p2 ?p3))
#       (not (placed ?p1))
#       (placed ?p2)
#       (placed ?p3)
#       (vertical ?p2)
#       (vertical ?p3))
#     :effect (and (empty_gripper ?gripper) 
#       (not (inGripper ?gripper ?p1))
#       (onDoublePlank ?p1 ?p2 ?p3)
#       (sideways ?p1)
#       (not (clearPlank ?p2))
#       (not (clearPlank ?p3))
#       (clearPlank ?p1)
#       (placed ?p1))
#   )

#   (:action putDown_plank_horizontal_onDoublePlank_both_sideways
#     :parameters (?gripper - gripper ?p1 - plank ?p2 - plank ?p3 - plank ?rob - robot)
#     :precondition (and
      
#       (inGripper ?gripper ?p1)
#       (not (= ?p1 ?p2))
#       (not (= ?p1 ?p3))
#       (not (= ?p2 ?p3))
#       (not (placed ?p1))
#       (placed ?p2)
#       (placed ?p3)
#       (sideways ?p2)
#       (sideways ?p3))
#     :effect (and (empty_gripper ?gripper) 
#       (not (inGripper ?gripper ?p1))
#       (onDoublePlank ?p1 ?p2 ?p3)
#       (horizontal ?p1)
#       (not (clearPlank ?p2))
#       (not (clearPlank ?p3))
#       (clearPlank ?p1)
#       (placed ?p1))
#   )

#   (:action putDown_plank_sideways_onDoublePlank_both_sideways
#     :parameters (?gripper - gripper ?p1 - plank ?p2 - plank ?p3 - plank ?rob - robot)
#     :precondition (and
      
#       (inGripper ?gripper ?p1)
#       (not (= ?p1 ?p2))
#       (not (= ?p1 ?p3))
#       (not (= ?p2 ?p3))
#       (not (placed ?p1))
#       (placed ?p2)
#       (placed ?p3)
#       (sideways ?p2)
#       (sideways ?p3))
#     :effect (and (empty_gripper ?gripper) 
#       (not (inGripper ?gripper ?p1))
#       (onDoublePlank ?p1 ?p2 ?p3)
#       (sideways ?p1)
#       (not (clearPlank ?p2))
#       (not (clearPlank ?p3))
#       (clearPlank ?p1)
#       (placed ?p1))
#   )
# )
# """



def get_hint_gpt(inputs):
    (domain, domain_string, problem_string, init_state, full_hint) = inputs
    domain = domain.split("./modules/")[1]
    msg = format_gpt_hint_prompt(domain, domain_string, problem_string,init_state, full_hint)
    print("GPT MESSAG=============================================================================")
    print(msg)    
    # return msg
    return _call_gpt(msg)



def format_gpt_hint_prompt(domain, domain_string, problem_string, init_state, full_hint):
    msg = """ This is the pddl domain file for the %s domain :

    %s

    A user has to solve a this problem task described in pddl 
    %s

    The plan was run till the problem reached this state - that is the set of predicates that are true :

    %s

    And the hint generated, which suggests which next action to take with certain arguments to actions replaced with ? is given below:

    %s

    Can you please convert the explanation into a brief, more non-expert friendly message that a novice user can understand.
    Also, can you suggested briefly what could be done to fix the issue , taking into account the state reached by the plan so far.

    """% (domain, domain_string, problem_string,init_state, full_hint)

    return msg

def format_gpt_prompt_for_explanation(domain_name, domain_pddl, problem_string,explanation_str, init_state):
    msg = """
The following lines describe the %s domain file
%s

The problem to be solved is described in pddl format  as:
%s

While running a plan for a problem, an action failed and an explanation generator was used
to generate the following explanation:

Explanation: %s

The state of the problem - which means the set of predicates that are true in the plan upto the first invalid plan are as follows 

State: %s

Can you please convert the explanation into a brief, more non-expert friendly message that a novice user can understand.
Also, can you suggested briefly what could be done to fix the issue , taking into account the state reached by the plan so far.

""" % (domain_name, domain_pddl, problem_string,explanation_str, init_state)
    
    return msg

def _call_gpt(msg):

    import requests
    import os
    API_KEY = os.get("OPENAI_API_KEY", None)
    key = API_KEY
    url = "https://api.openai.com/v1/chat/completions"
    headers = {"Authorization": f"Bearer {key}"}
    data = {'model': 'gpt-3.5-turbo', "messages": [{"role": "user", "content": msg}]}

    try:
        y = requests.post(url, headers=headers, json=data).json()
        return y["choices"][0]["message"]["content"]
    except Exception:

        return "GPT ERROR"

def get_gpt_human_description_for_explanation(domain_name, domain_pddl, problem_string,explanation_str, init_state):

    print("GPT")


    if explanation_str.get("err_code", "") == "BAD ACTION":

        gpt_msg = format_gpt_prompt_for_explanation(domain_name, domain_pddl, problem_string,
                                                    explanation_str.get("failure_cause", "DEBUG NO FAILURE"), 
                                                    init_state)

        return _call_gpt(gpt_msg)

    else:

        return "No errors"




if __name__ == "__main__":

    print("Hello")
    import os
    API_KEY = os.get("OPENAI_API_KEY", None)
    # x = openai.Model.list()
    msg ="""
The following lines of text describe a gripper PDDL domain
(define (domain gripper-strips)
   (:predicates (room ?r)
		(ball ?b)
		(gripper ?g)
		(at-robby ?r)
		(at ?b ?r)
		(free ?g)
		(carry ?o ?g))

   (:action move
       :parameters  (?from ?to)
       :precondition (and  (room ?from) (room ?to) (at-robby ?from))
       :effect (and  (at-robby ?to)
		     (not (at-robby ?from))))



   (:action pick
       :parameters (?obj ?room ?gripper)
       :precondition  (and  (ball ?obj) (room ?room) (gripper ?gripper)
			    (at ?obj ?room) (at-robby ?room) (free ?gripper))
       :effect (and (carry ?obj ?gripper)
		    (not (at ?obj ?room)) 
		    (not (free ?gripper))))


   (:action drop
       :parameters  (?obj  ?room ?gripper)
       :precondition  (and  (ball ?obj) (room ?room) (gripper ?gripper)
			    (carry ?obj ?gripper) (at-robby ?room))
       :effect (and (at ?obj ?room)
		    (free ?gripper)
		    (not (carry ?obj ?gripper)))))

The following lines denote the problem for this domain




(define (problem gripper-2)
(:domain gripper-strips)
(:objects  rooma roomb left right ball1 ball2 )
(:init
(room rooma)
(room roomb)
(gripper left)
(gripper right)
(ball ball1)
(ball ball2)
(free right)
(at ball2 rooma)
(at-robby rooma)
(carry ball1 left)
)
(:goal
(and
(at ball1 roomb)
(at ball2 roomb)
)
)
)

The action (pick ball1 left rooma) action was executed from the current state and failed.
The compact explanation generated was that 
Explanation:  (pick ball1 left rooma) failed because (free left)  is not true.

Question: Can you please convert the explanation above into something a human who does not have a great understanding of ai or planning could understand better?
Ideally, the response should be geared towards someone who does not have a formal education in computer science and should be verbose. 
Using the domain above, can you please provide an alternate action that might cause this action to succeed and also explain your reasoning.
    """

    import requests
    key = API_KEY
    url = "https://api.openai.com/v1/chat/completions"
    headers = {"Authorization": f"Bearer {key}"}
    data = {'model': 'gpt-3.5-turbo', "messages": [{"role": "user", "content": msg}]}
    y = requests.post(url, headers=headers, json=data).json()
    print(y["choices"][0]["message"]["content"])
