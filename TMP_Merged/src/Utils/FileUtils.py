
def read(filename):
    with getHandle(filename,"r") as fh:
        data = fh.read()
    return data

def write(filename, data):
    with getHandle(filename,"w") as fh:
        fh.write(data)

def getHandle(filename, mode):
    return open(filename, mode)


def getStringIOFile(str):
    import StringIO
    strPlanFileH = StringIO.StringIO()
    strPlanFileH.write(str)
    strPlanFileH.seek(0)
    return strPlanFileH

def writeStringIOFile(file,location):
    with getHandle(location, 'w') as fd:
        fd.write(file.getvalue())

