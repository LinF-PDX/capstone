import os
import can

#os.system('sudo ip link set can0 type can bitrate 100000')
#os.system('sudo ifconfig can0 up')
os.system('sudo ip link set can1 type can bitrate 100000')
os.system('sudo ifconfig can1 up')

#can0 = can.interface.Bus(channel = 'can0',interface='socketcan')
can1 = can.interface.Bus(channel = 'can1', interface='socketcan')
#msg=can.Message(is_extended_id = False, arbitration_id=0x123,data=[0,1,2,3,4,5,6,7])


#can0 = can.interface.Bus(channel = 'can0', interface= 'socketcan_ctypes')
#msg = can.Message(arbitration_id=0x123, data=[0, 1, 2, 3, 4, 5, 6, 7], extended_id=False)


msg= can1.recv(10.0)#receive(time out)
#can0.send(msg)#sen
print(msg)
if msg is None:
    print('Timeout, no msg')

#time.sleep(1)
#os.system('sudo ifconfig can0 down')
#time.sleep(1)
#os.system('sudo ifconfig can1 down')


