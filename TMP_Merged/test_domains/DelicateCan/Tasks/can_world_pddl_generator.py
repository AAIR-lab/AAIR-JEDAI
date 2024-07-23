def write_objects_cans(fhandle,number_of_cans):
    s = ""
    for i in range(number_of_cans):
        s += "object{}".format(i) + " "
    s += "- object \n"
    fhandle.write(s)

def write_objects_cans_inti_loc(fhandle,number_of_cans):
    s = ""
    for i in range(number_of_cans):
        s += "obj_loc{}".format(i) + " "
    s += "- location \n"
    s += "table - location \n"
    fhandle.write(s)

def write_objects_cans_gp(fhandle,number_of_cans):
    s = ""
    for i in range(number_of_cans):
        s += "gp_ob{} ".format(i)
    s += "- pose \n"
    fhandle.write(s)

def write_objects_cans_pdp(fhandle,number_of_cans):
    s = ""
    for i in range(number_of_cans):
        s += "pd_ob{} ".format(i)
    s += "- pose\n"
    fhandle.write(s)

def write_objects_robot_init_loc(fhandle):
    fhandle.write("defaultloc - location\n")
    fhandle.write("initpose - pose\n")
    fhandle.write("traj - trajectory\n")
    fhandle.write("initloc - location\n")
    fhandle.write("gripper - manip\n")


def write_objects(fhandle,numbre_of_cans):
    fhandle.write("(:objects\n")
    write_objects_cans(fhandle,numbre_of_cans)
    write_objects_cans_inti_loc(fhandle,numbre_of_cans)
    write_objects_cans_gp(fhandle,numbre_of_cans)
    write_objects_cans_pdp(fhandle,numbre_of_cans)
    write_objects_robot_init_loc(fhandle)
    fhandle.write(")\n")

def write_init_at(fhandle,number_of_cans):
    for i in range(number_of_cans):
        fhandle.write("(at object{} obj_loc{})\n".format(i,i))

def write_init_isgp(fhandle,number_of_cans):
    for i in range(number_of_cans):
        fhandle.write("(isgp gp_ob{} object{} gripper)\n".format(i,i))

def write_init_ispd(fhandle,number_of_cans):
    for i in range(number_of_cans):
        fhandle.write("(ispd pd_ob{} object{} gripper)\n".format(i,i))

def write_init_misc(fhandle,number_of_cans):
    fhandle.write("(clear table)\n(empty gripper)\n(robotat initloc)\n(robotpose initpose)\n")


def write_init(fhandle,number_of_cans):
    fhandle.write("(:init\n")
    write_init_at(fhandle,number_of_cans)
    write_init_isgp(fhandle,number_of_cans)
    write_init_ispd(fhandle,number_of_cans)
    # write_init_ismp(fhandle,number_of_cans)
    write_init_misc(fhandle,number_of_cans)
    fhandle.write(")\n")

def write_goal(fhandle):
    fhandle.write("(:goal (and\n(at object1 table)\n))\n")

def generate_pddl(number_of_cans):
    fhandle = open("new_canworld_{}_cans_problem.pddl".format(number_of_cans),"w")
    fhandle.write("(define (problem p01)\n")
    fhandle.write("(:domain canworld)\n")
    write_objects(fhandle,number_of_cans)
    write_init(fhandle,number_of_cans)
    write_goal(fhandle)
    fhandle.write(")\n")
    fhandle.close()


generate_pddl(2)
