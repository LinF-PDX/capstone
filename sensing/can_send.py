import os
import can
import time

os.system('sudo ip link set can0 type can bitrate 500000')
os.system('sudo ifconfig can0 up')
#os.system('sudo ip link set can1 type can bitrate 100000')
#os.system('sudo ifconfig can1 up')

can0 = can.interface.Bus(channel = 'can0',interface='socketcan')
#can1 = can.interface.Bus(channel = 'can1', interface='socketcan')
data1= [0,1,2,3,4,5,6,7]

while (1):
    time.sleep(0.017)
    msg=can.Message(is_extended_id = False, arbitration_id=0x123,data=data1)
    #msg1= can.Message(is_extended_id =False, arbitration_id=0x123,data=[23,141,23,132])
    #data1 = data1 *2
#can0 = can.interface.Bus(channel = 'can0', interface= 'socketcan_ctypes')
#msg = can.Message(arbitration_id=0x123, data=[0, 1, 2, 3, 4, 5, 6, 7], extended_id=False)


#can1.recv(10.0)#receive(time out)
    can0.send(msg)#sen
    #can0.send(msg1)

#time.sleep(1)
os.system('sudo ifconfig can0 down')
#time.sleep(1)
#os.system('sudo ifconfig can1 down')


