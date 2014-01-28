from multiprocessing import Process, Queue, Value

maxqsize=100
commandq=Queue()    #contains items in format (user, field, data)

def addtoqueue(dispatch):
    if (commandq.qsize()<=maxqsize):
        commandq.put(dispatch)
