
def write_goal(goal_domino,domain,problem):
    s = "(picked domino"+str(goal_domino)+")"
    domain = domain.replace("###GOAL_HERE###",s)
    problem = problem.replace("###GOAL_HERE###",s)
    return domain,problem

def write_objects(n,problem):
    s = ""
    for i in range(n):
        s += "domino"+str(i)+" "
    s += "- domino"
    problem = problem.replace("###DOMINOS_HERE###",s)
    return problem

def write_prob_effects(n,k,goal_domino,domain):
    s = ""
    for i in range(k):
        for j in range(2):
            domino_number = (2 * (i+1) + j)
            p = (int((100.0 / float(i+2)) * 100) / 100.0) / 100.0
            s += "(probabilistic " + "\n"
            s += str(p) + " (dropped domino"+str(domino_number)+")" + "\n"
            s += ")\n"
    domain = domain.replace("###PROBABILISTIC_EFFECTS_HERE###",s)
    return domain


if __name__ == "__main__":
    n = 15
    k = 2
    goal_domino = 1
    domain = open("domain_skeleton.ppddl","r").read()
    problem = open("problem_skeleton.ppddl","r").read()
    domain,problem = write_goal(goal_domino,domain,problem)
    domain = write_prob_effects(n,k,goal_domino,domain)
    problem = write_objects(n,problem)
    new_domain = open("domain.ppddl","w")
    new_problem = open("problem.ppddl","w")
    new_domain.write(domain)
    new_problem.write(problem)
    new_domain.close()
    new_problem.close()
