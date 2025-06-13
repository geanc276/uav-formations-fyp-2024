from gps import GPSCoord
from geometry_msgs.msg import Pose




class DronePos: 
    def __init__(self) -> None:
        self.local = Pose()
        self.global_pos = GPSCoord(0, 0, 0)
        self.heading = 0


    def __str__(self) -> str:
        return f'X{self.local.position.x:8.3f}\tY{self.local.position.y:8.3f}\tZ{self.local.position.z:8.3f}\t\tN{self.global_pos.lat:3.8f}\tE{self.global_pos.long:3.8f}\tZ{self.global_pos.alt:3.8f}'
