import socket    
import json     
import time

class RobotConnector():    
    def __init__(self, ip_address='192.168.123.161', port=8000):     
        address_server = (ip_address, port)     
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    
        self.s.connect(address_server)     

    def robot_high_control(self, mode=2, gaitType=1, speedLevel=0, 
                      footRaiseHeight=0, bodyHeight=0, 
                      euler=[0,0,0], velocity=[0,0], yawSpeed=0.0, reverve=0):    
        data = [mode, gaitType, speedLevel, footRaiseHeight, bodyHeight, euler, velocity, yawSpeed, reverve]  
        data = json.dumps(data)  
        self.s.send(bytes(data.encode('utf-8')))   
