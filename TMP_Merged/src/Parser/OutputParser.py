from src.States.PDDLState import *
import re,sys,pdb, copy
propPattern = re.compile("\(.*\)\Z")


def tryOpen(fname, mode):
    try:
        fhandle = open(fname, mode)
    except IOError as e:
        print "\nWhile opening {0}".format(fname)
        print "Encountered IO Error {0}: {1}".format(e.errno, e.strerror)
        print "Ending process. \n"
        sys.exit(-1)

    return fhandle


def tryIO(fname, mode, strBufPtr=""):
    '''tries to open file in appropriate mode and do I/O.
    mode can be "read" or "write". 
    returns the read buffer if mode is read, and return value of f.write() 
    if mode is write.''' 

    try:
        if mode == "read":
            fhandle = open(fname, "r")
            strBufPtr = fhandle.read()
            retVal = strBufPtr
        if mode == "write":
            fhandle = open(fname, "w")
            retVal = fhandle.write(strBufPtr)
    except IOError as e:
        print "While working on {0}".format(fname)
        print "Encountered IO Error {0}: {1}".format(e.errno, e.strerror)
        print "Ending process. \n"
        sys.exit(-1)

        
    return retVal
        

class OutputParser:
    def __init__(self, strOutput, plannerName = None, planCount = None):
        self.strOutput = strOutput
        self.stateList = []#contains partial states. 
        self.propSet = set()
        self.FDPlanCount = planCount
                  
        if plannerName == "ff":
            self.parseFFOutput()
        if plannerName == "fd":
            self.parseFDOutput()
            
        
    def getPartialStatesList(self):
        return copy.copy(self.stateList)
    

    def parseFDOutput(self):
        "Planner mode for parsing: FD"
        relevantPrefixStr = ""
        relevantSegment = ""
        stateList = []
        
        if not "Solution found!" in self.strOutput:
            print "Solution not found. Error"
            sys.exit(-1)
     
        if "End state list" in self.strOutput:
            stateListSegments = self.strOutput.split("End state list")
            print "Found "+repr(len(stateListSegments)-1) + " state lists"
            print "using list #" + repr(self.FDPlanCount)
            relevantSegment = stateListSegments[self.FDPlanCount-1].split("Begin state list")[1]
        else:
            print "Output from planner garbled"
            # pdb.set_trace()
     
        #get pruned atoms
        prunedList = self.strOutput.partition("Translating task:")[0].split("\n")
        pruneLines = filter(lambda x: "pruned" in x and "=" not in x, prunedList)
        prunedFacts = [s.replace("pruned static init fact: Atom ", "") for s in pruneLines]
        constantState = self.getStateFromStr("\n".join(prunedFacts))
        
        for atomStateStr in relevantSegment.split(stateDelimiter):
            stateStr = atomStateStr.replace("Atom ", "").strip()
            if len(stateStr)>0:
                s = self.getStateFromStr(stateStr)
                if s.size() >0:
                    s.patch(constantState)
                    stateList.append(s)
        self.stateList = stateList
        return stateList
        

    def parseFFOutput(self):
        ''' returns list of states from self.strOutput.
        First state in the list is the state before 
        the first action.'''
        relevantStr = self.strOutput.split(stateListDelimiter)[1]
        stateList = []
        
        for stateStr in relevantStr.split(stateDelimiter):
            s = self.getStateFromStr(stateStr)
            if s.size() >0:
                stateList.append(s)
        self.stateList = stateList
        return stateList


    def getStateFromStr(self, stateStr):
        s = PDDLState()
        for rawPropositionStr in stateStr.split("\n"):
            propositionStr = rawPropositionStr.strip().replace(\
                "(", " ").replace(")", " ").replace(",", " ").lower()
            propositionStr = "("+propositionStr.strip()+")"
            if propositionStr != "()":
                if propPattern.match(propositionStr) != None:
                    s.addProposition(propositionStr)
                else:
                    print "not a proposition: " + propositionStr
        return s


    def getPropSet(self):
        stateProps = self.stateList[0].getAllProps()
        sProps = stateProps
        for state in self.stateList:
            prevStateProps = stateProps
            stateProps = state.getAllProps()
            diff = prevStateProps^stateProps 
            # if len(diff) >0:
            #     print "diffs from previous:"
            #     print "diff: "+ str(diff)+"\n\n"
            sProps = sProps|stateProps

        return sProps

    def getStateByIndex(self, index):
        if len(self.stateList) > index:
            return self.stateList[index]
        else:
            print "Error: state index out of range"
            return -1

    def getFFPlan(self):
        ffPlanStr = self.strOutput.split("found legal plan as follows")[1].\
            split("time")[0]
        return ffPlanStr.replace("step", "")


                
            
        

        
    
