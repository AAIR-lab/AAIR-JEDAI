#!/usr/bin/env python3

import pddlpy
import argparse
import sys
import pickle 


parser = argparse.ArgumentParser()
parser.add_argument('--domain', type=str, help='PDDL Domain File')
parser.add_argument('--problem', type=str, help='PDDL problem File')
parser.add_argument('--action', type=str, help='Name of the action and arguments')
args = parser.parse_args()




def parser(domainFile, problemFile, action = None):
	action = args.action.split()[0]
	list_arguments = args.action.split()[1:]
	

	domainProblem = pddlpy.DomainProblem(domainFile, problemFile)
	universeObjects = domainProblem.worldobjects()
	predicates = domainProblem.initialstate()
	
	pickle.dump(universeObjects, open( "objects_in_the_universe.p", "wb" ), protocol=2 )
	pickle.dump(predicates, open("predicates.p","wb"), protocol=2)

	#Action Definition - Precondition, effect, argument
	actionObject = list(domainProblem.ground_operator(action))
	actionObjectVaribaleList = domainProblem.domain.operators[action]

	
	#Precondition
	pos_precondition = list(actionObject[0].precondition_pos)
	neg_precondition = list(actionObject[0].precondition_neg)
	#pos_precondition = list(actionObjectVaribaleList.precondition_pos)
	#neg_precondition = list(actionObjectVaribaleList.precondition_neg)
	
	#print(domainProblem.domain.operators[action].precondition_pos)
	pickle.dump(pos_precondition, open("pos_precondition.p", "wb"), protocol=2)
	pickle.dump(neg_precondition, open("neg_precondition.p", "wb"), protocol=2)


	#Effect
	pos_effect = list(actionObject[0].effect_pos)
	neg_effect = list(actionObject[0].effect_neg)
	pickle.dump(pos_effect, open("pos_effect.p", "wb"), protocol=2)
	pickle.dump(neg_effect, open("neg_effect.p", "wb"), protocol=2)
	
	#Argument
	argument = []
	for item in list_arguments:
		argument.append([None, universeObjects[item], item])
	pickle.dump(argument, open("argument.p", "wb"), protocol=2)

	
	



if __name__ == '__main__':
	parser(args.domain, args.problem, args.action)
	#sys.stdout.write(str(list_objects))