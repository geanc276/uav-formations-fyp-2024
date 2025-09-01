import mmap
import ctypes
import signal
import time

# CTypes structure 
class NodeIdentifier(ctypes.Structure):
    _fields_ = [("identifier", ctypes.c_char * 18)]

class DroneName(ctypes.Structure):
    _fields_ = [("name", ctypes.c_char * 5)]

class SRPSequenceNumber(ctypes.Structure):
    _fields_ = [("sequenceNumber", ctypes.c_uint32)]

class SafetyData(ctypes.Structure):
    _fields_ = [
        ("position", ctypes.c_int16*3),
        ("speed", ctypes.c_uint8),
        ("direction", ctypes.c_int16*3),
        ("rotation", ctypes.c_int16*3)
    ]

class TimeStamp(ctypes.Structure):
    _fields_ = [("time", ctypes.c_double)]

class NeighbourTableEntry(ctypes.Structure):
    _fields_ = [
        ("id", NodeIdentifier),
        ("name", DroneName),
        ("seq", SRPSequenceNumber),
        ("sd", SafetyData),
        ("time", TimeStamp),
        ("padding", ctypes.c_uint8*69) # Empty always (mutex bytes + 30 bytes padding)
    ]

    def __new__(cls, buf):
        return cls.from_buffer(buf)
    def __init__(*args):
        pass

class NeighbourTable():
    tableEntries = dict()
    shm = None

    def __init__(self):
        pass

    def readMMap(self):
        # Identify each UAV stamp
        uavs = ["uav0mutex", "uav1mutex", "uav2mutex", "uav3mutex", "uav4mutex"]
        for uav in uavs:
            ind = self.shm.find(bytes(uav, "utf-8"))
            if (ind != -1):
            
                struct = NeighbourTableEntry.from_buffer_copy(self.shm[ind-200:ind-72])
                self.tableEntries[str(struct.name.name)] = struct
                print(struct.id.identifier)
                print("Seq, Name: " + str(struct.seq.sequenceNumber) + ", " + str(struct.name.name)) 
        
        return

    def signalHandler(self, signum, frame):
        try:
            with open("/neighbourtable", "r+b") as f:        
                self.shm = mmap.mmap(f.fileno(), 0, mmap.MAP_SHARED, mmap.PROT_WRITE)
                self.readMMap()

                # Act on table entries here
                print("nt entries: " + str(self.tableEntries.keys()))

                self.shm.close()
                f.close()
        except ValueError as ex:
            print("Exception occured: " + str(ex))
            if self.shm:
                self.shm.close()

if __name__ == '__main__':
    table = NeighbourTable()
    
    # Same as beaconing frequency
    signal.setitimer(signal.ITIMER_REAL, 1, 1.0)
    signal.signal(signal.SIGALRM, table.signalHandler)

    start = time.time()

    while signal.getitimer(signal.ITIMER_REAL): 
        if time.time() - start > 20: # Temp timeout handle this better
            break
            


