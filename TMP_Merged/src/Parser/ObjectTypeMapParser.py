import Config
class ObjecTypeMapParse(object):
    def __init__(self,problem_file,domain_file):
        self.problem_file = open(problem_file,"r")
        self.domain_file = open(domain_file,"r")


    def get_object_map(self):
        object_str = self.problem_file.read().split(":objects")[1]
        object_str = object_str.split(")")[0]
        object_str = object_str.strip()
        map = {}
        lines = object_str.split("\n")
        for line in lines:
            temp = line.split("-")
            object_type = temp[-1].strip()
            for object in temp[0].split():
                #map[object.strip().lower()] = object_type.lower()
                if object_type.lower() in map:
                    map[object_type.lower()].append(object.strip().lower())
                else:
                    map[object_type.lower()] = [object.strip().lower()]
        object_str = self.domain_file.read()
        if "constants" in object_str:
            object_str = object_str.split(":constants")[1]
            object_str = object_str.split(")")[0]
            object_str = object_str.strip()
            lines = object_str.split("\n")
            for line in lines:
                temp = line.split("-")
                object_type = temp[-1].strip()
                for object in temp[0].split():
                    # map[object.strip().lower()] = object_type.lower()
                    if object_type.lower() in map:
                        map[object_type.lower()].append(object.strip().lower())
                    else:
                        map[object_type.lower()] = [object.strip().lower()]
        return map


if __name__ == "__main__":

    parser = ObjecTypeMapParse(Config.PROJ_DIR+"SampleTasks/canworld_10_cans_problem.pddl")
    print parser.get_object_map()