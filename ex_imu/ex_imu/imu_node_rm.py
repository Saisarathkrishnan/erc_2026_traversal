import rclpy
from rclpy.node import Node
from custom_msgs.msg import ImuData
from geometry_msgs.msg import Vector3

import socket
import time
import pickle
import os
import serial


class BNO055Node(Node):
    def __init__(self):
        super().__init__('rm_auto')

##### filler
        self.ip = "10.0.0.7"
        self.port = 5005
        self.sock = None
        self.server_addr = (self.ip, self.port)
        self.initialize_sock()
        self.msgR="M0X26Y75P0Q0A0S0R0D0E0"
        self.msg="M0X26Y75P0Q0A0S0R0D0E0"
        self.p=0
#####
        print("inf")

        self.l1=dict({"r":400.0,"p":400.0,"y":400.0})
        self.l2=dict({"r":400.0,"p":400.0,"y":400.0})
        self.toAnglel1=80
        self.toAnglel2=-80

        self.link1Pwm=0
        self.link2Pwm=0
#        self.l2=dict()
        self.line="123"
        
        self.imu_pub_ = self.create_publisher(ImuData, '/imu_data', 10)

        self.initialize_imu()
        self.timer_ = self.create_timer(0.01, self.publish_imu)
#        self.timer_ = self.create_timer(0.01, self.send)

    def initialize_imu(self):
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)
        time.sleep(1)
        if(self.serial_port):
            print("into the serial port1 saf asf")
        else:
            print("serial port issue")
        self.serial_port.dtr = False   # GPIO0 HIGH
        self.serial_port.rts = True    # EN LOW (reset)
        time.sleep(0.1)

        self.serial_port.rts = False   # EN HIGH (run)
        time.sleep(0.1)
        self.serial_port.dtr = True 
        self.serial_port.close()
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)
        time.sleep(1)
        if(self.serial_port):
            print("into the serial port reseted")
        else:
            print("serial port issue")


    def initialize_sock(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print("UDP socket initialized")
        except socket.error as e:
            print("Socket error:", e)
            self.sock = None
        
    def send(self):
        #print(self.msg)
        if self.sock is None:
            print("Socket not initialized")
            return

        try:
            data = self.msg.encode("utf-8")
            self.sock.sendto(data, self.server_addr)
        except socket.error as e:
            print("Send error:", e)

    
    def publish_imu(self):
        
        self.line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
        x=self.line
        if(len(self.line)!=0 and str(self.line[0])=="y"):
            print(self.line)

            y=type(self.line)
            z=x.split('_')
            self.l1["r"]=float(z[2])
            self.l1["p"]=-float(z[3])
            self.l1["y"]=float(z[1])

            self.l2["r"]=float(z[6])
            self.l2["p"]=float(z[7])
            self.l2["y"]=float(z[5])
            
            result=self.msgR
            self.msg=result
            self.send()





            

'''
        if(self.line[0]!="2"):
            self.initialize_imu()
        else: 
            data=self.line[2:-1]
            xcv=data.split("_")
            for i in xcv:
                xcv[xcv.index(i)]=float(i)
            
            
            self.yaw=xcv[0]
            self.pitch=xcv[1]
            self.roll=xcv[2]
            print(f"r: {self.roll:7.2f} p: {self.pitch:7.2f} y: {self.yaw:7.2f}", end='\r')

            imu_msg_ = ImuData()
            imu_msg_.orientation.x = self.roll
            imu_msg_.orientation.y = self.pitch
            imu_msg_.orientation.z = self.yaw

            imu_msg_.acceleration.x = 0.0
            imu_msg_.acceleration.y = 0.0
            imu_msg_.acceleration.z = 0.0

            # Publish
            self.imu_pub_.publish(imu_msg_)
'''


def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_calibration()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
