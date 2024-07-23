import time
import sys
import subprocess
import os
import FileUtils

def executeCommand(cmd, successStr, outputFname, pollTime=2):
    ''' A generic utility for executing a command.
    outputFname stores stdout and stderr'''
    initTime = time.time()
    print "Executing %s...   " % (cmd),
    sys.stdout.flush()
    ## Using subprocess.call so that all the messages from the planner
    ## can be captured and parsed for the string "Solution found!"

    dumpFile = FileUtils.getHandle(outputFname, "w")
    p = subprocess.Popen([cmd], shell=True, stdout=dumpFile, stderr=dumpFile)

    startTime = time.time()
    # p = subprocess.Popen([cmd], shell = True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    while p.poll() == None:
        # time.sleep(pollTime)
        if time.time() - startTime > 350:
            # ans = raw_input("Still running. Continue (c), Restart (r), Quit(q)? ")
            ans = "q"
            print "Killing process"
            startTime = time.time()
            if ans.strip() == "q":
                os.kill(p.pid + 1, 9)
                sys.exit(-1)
            elif ans.strip() == "c":
                continue
            elif ans.strip() == "r":
                os.kill(p.pid + 1, 9)
                return executeCommand(cmd, successStr)
            else:
                print "Unrecognized response. Continuing."

    dumpFile.close()
    # time.sleep(5)
    endTime = time.time()

    msg = FileUtils.read(outputFname)
    # out, err = p.communicate()
    # msg = out+err

    print "\nPlanning time: {0}".format(endTime - initTime)

    if successStr in msg:
        print "Success!"
        print
        return msg
    else:
        print "Failure... Planner message:"
        print msg
        return -1
