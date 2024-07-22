import re, pdb
from src.States.State import State
import copy


class PDDLState(State):
    def __init__(self, trueSet=None, falseSet=None):
        self.__trueSet = set()
        self.__falseSet = set()
        if type(trueSet) == set:
            self.__trueSet = trueSet
        if type(falseSet) == set:
            self.__falseSet = falseSet
        self.__objDict = {}

    def __deepcopy__(self, memodict={}):
        new_pddl_state = PDDLState()
        new_pddl_state.__trueSet = copy.deepcopy(self.__trueSet)
        new_pddl_state.__falseSet = copy.deepcopy(self.__falseSet)
        new_pddl_state.__objDict = copy.deepcopy(self.__objDict)
        return new_pddl_state


    def setObjDict(self, d):
        self.__objDict = d
    
    def getObjDict(self):
        return self.__objDict
        
    def addProposition(self, propStr):
        r1 = re.compile("\(not-", re.IGNORECASE)
        r2 = re.compile("\(not ", re.IGNORECASE)
        propStr = re.sub("\s+", " ", propStr).strip()
        
        if (r1.match(propStr) != None):
            self.addFalse(r1.sub("(", propStr.strip()))
        elif (r2.match(propStr) != None):
            toAdd = r2.sub("(", propStr.strip())
            toAdd= toAdd.replace("((", "(").replace("))", ")")
            self.addFalse(toAdd)
        else:
            self.addTrue(propStr.strip())
    
    def addProps(self, propList):
        ''' add propositions from list of prop strs'''
        for propStr in propList:
            self.addProposition(propStr)
            
    def addTrue(self, propStr):
        self.__trueSet.add(propStr)
        self.__falseSet.discard(propStr)

    def addFalse(self, propStr):
        self.__falseSet.add(propStr)
        self.__trueSet.discard(propStr)

    def addTrueProps(self, propList):
        for propStr in propList:
            self.addTrue(propStr)

    def getTrueProps(self):
        return self.__trueSet

    def getFalseProps(self):
        return self.__falseSet

    def getAllProps(self):
        return self.getTrueProps()|self.getFalseProps()

    def printState(self):
        # print("False: ")
        # if len(self.__falseSet)>0:
        #     l = list(self.__falseSet)
        #     l.sort()
        #     print "\n".join(l)
        print("True: ")
        if len(self.__trueSet)>0:
            l = list(self.__trueSet)
            l.sort()
            print "\n".join(l)
        
        print
        print
    
    def size(self):
        return len(self.__trueSet) + len(self.__falseSet)

    def patch(self, deltaState):
        for prop in deltaState.getFalseProps():
            self.__trueSet.discard(prop)

        for prop in deltaState.getTrueProps():
            self.__trueSet.add(prop)
            self.__falseSet.discard(prop)

    def removeTrueProp(self, prop):
        self.__trueSet.discard(prop)

    def removeFalseProp(self, prop):
        self.__falseSet.discard(prop)

    def makeCWAExplicit(self, propSet):
        for prop in propSet:
            if prop not in self.__trueSet:
                self.__falseSet.add(prop)
                
    def purgeFactsAbout(self, propString):
        trueProps = self.getTrueProps()
        falseProps = self.getFalseProps()
        for prop in trueProps:
            if symbol in prop:
                self.removeTrueProp(prop)

        for prop in falseProps:
            if symbol in prop:
                self.removeFalseProp(prop)

    def getStateCopy(self):
        return PDDLState(self.getTrueProps().copy(), self.getFalseProps().copy())

    def rawStringForm(self):
      result = ""
      if len(self.__trueSet) > 0:
        l = list(self.__trueSet)
        l.sort()
        result += "\n".join(l)
      return result

    def reset_state(self):
        self.__trueSet = set()
        self.__falseSet = set()

    def __contains__(self, item):
        l = list(self.__trueSet)
        for prop in l:
            if item in prop:
                return True
        return False