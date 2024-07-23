import copy
from Config import DOMAIN, TEST_DIR_NAME
import importlib
from src.DataStructures.Generator import Generator

class ArgumentV2(object):

    def __init__(self, name, arg_type, generator_class_name, name_alias=None,generator_object=None,executor = None):
        self.name = name
        self.name_alias = name_alias
        self.type = arg_type
        self.generator_class_name = generator_class_name
        self.generator_object = generator_object
        self.value = None
        self.executor = copy.deepcopy(executor)

    def apply(self,simulator,other_generated_values):
        if self.executor is not None:
            self.executor.apply(simulator,self.value,other_generated_values)

    def execute(self,simulator,other_generated_values):
        if self.executor is not None:
            self.executor.execute(simulator,self.value,other_generated_values)

    def has_alternate_values(self):
        # if self.generator_object is None:
        #     import IPython
        #     IPython.embed()
        # result = self.generator_class_name is not None and self.generator_object.type != Generator.Generator.TYPE_QUERY_GENERATOR
        result = False
        if self.generator_class_name is not None:
            if self.generator_object is not None:
                if self.generator_object.type != Generator.TYPE_QUERY_GENERATOR:
                    result = True
        print self.name+" "+str(result)
        return result

    def get_generator(self, known_argument_values, low_level_state):
        if self.generator_object is not None:
            return self.generator_object
        else:
            if self.generator_class_name is not None:
                print self.generator_class_name
                # import IPython
                # IPython.embed()
                try:
                    gen_class  =  importlib.import_module( TEST_DIR_NAME + "." +DOMAIN+'.Generators.'+self.generator_class_name)
                    self.generator_object = getattr(gen_class, self.generator_class_name)(ll_state=low_level_state, known_argument_values=known_argument_values)
                except Exception as e:
                    print ""
                    print "####  Something went wrong with Generator "+self.generator_class_name
                    print e.message
                    import traceback
                    print(traceback.format_exc())
                    exit(-1)


            else:
                self.generator_object = None
        return self.generator_object

    def reset_generator(self):
        self.generator_object = None

    def __deepcopy__(self, memodict={}):
        executor = copy.deepcopy(self.executor)
        return ArgumentV2(copy.deepcopy(self.name),
                          copy.deepcopy(self.type),
                          copy.deepcopy(self.generator_class_name),
                          copy.deepcopy(self.name_alias),
                          executor=executor)

    def __repr__(self):
        return "Argument Name: "+self.name + " of Type :" + self.type + " and Generator Name: " + str(self.generator_class_name)
