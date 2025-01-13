from backend import Sensing, can_send, can_init,can_receive, add_parser_arguments, can_initvalue, can_down
import cv2
import logging
import argparse
import multiprocessing as mp
import time
from datetime import datetime
import os
import csv
import RPi.GPIO as GPIO


pin = 27
S_Enable = False
Comm_Process = None
Sens_Process = None
Total_Travel = 100 #100m
BUFFER_SIZE = 10
stop = mp.Event()
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
parser = argparse.ArgumentParser()
add_parser_arguments(parser)
args = parser.parse_args()

def Connect_To_GUI():
    pass

def Send_To_GUI():
    pass

def sense_dot(conn,args):
    dis = Sensing(ActualBoardWidth=args.actualboardwidth,laser_color=args.lasercolor,gpu=args.gpu)
    dis.camera_setup()
    if not dis.cap.isOpened():
        logger.error("Error: Could not open video file.")
        exit()
    
    while not stop.is_set():
        ret, frame = dis.cap.read()
    
        if not ret:
            break
        
        # Display the frame
        new_width = int(frame.shape[1] * 0.5)
        new_height = int(frame.shape[0] * 0.5)
        frame = cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
        
        dis_off = dis.off_dis(frame)
        if dis_off == "error":
            logger.error("Error detecting the cross")
            conn.send(10.0)
        else:
            logger.info("Distance off the track is "+str(dis_off)+" mm")
            conn.send(dis_off)

    dis.cap.release()
    #cv2.destroyAllWindows()

def communication(conn,can0,can1,args):
    if not os.path.exists("data"):
        os.makedirs("data")
    now = datetime.now()
    path = "data/" + str(now.year) + "_" + str(now.month) + "_" + str(now.day) + "_" + str(now.hour) + "_" + str(now.minute) + "_" + str(now.second) + ".csv"
    with open(path, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Travel_Distance", "Height_Difference"])
    buffer = []
    while not stop.is_set():
        if conn.poll():
            data = conn.recv()
            can_send(int(data),can0)
        Travel_Distance, Height_Difference = can_receive(can1)
        if Travel_Distance == None and Height_Difference == None:
            pass
        else:
            buffer.append([Travel_Distance, Height_Difference])
            if len(buffer) >= BUFFER_SIZE:
                with open(path, mode="a", newline="") as file:
                    writer = csv.writer(file)
                    writer.writerows(buffer)
                buffer.clear()
                # Send_To_GUI(csv)
            if float(Travel_Distance) >= 100.0:
                stop.set()
    if len(buffer) > 0:
        with open(path, mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerows(buffer)

def button_callback(channel):
    global S_Enable, Comm_Process, Sens_Process, stop, args
    ### INITIALIZE A LED to demonstate the profilograph is Running Or Not
    if S_Enable == True:
        stop.set()
        if Sens_Process.is_alive():
            Sens_Process.join(timeout=5)
            if Sens_Process.is_alive():
                Sens_Process.terminate()
        if Comm_Process.is_alive():
            Comm_Process.join(timeout=5)
            if Comm_Process.is_alive():
                Comm_Process.terminate()
        Sens_Process = None
        Comm_Process = None
        can_down()
        S_Enable = False
        logger.info("Closed")
    else: 
        stop.clear()
        conn1, conn2 = mp.Pipe()
        can0, can1 = can_init()
        S_Enable = True
        can_initvalue(args,S_Enable,can0)
        Sens_Process = mp.Process(target=sense_dot, args=(conn1,args,))
        Comm_Process = mp.Process(target=communication, args=(conn2,can0,can1,args,))
        Sens_Process.start()
        Comm_Process.start()
        sens_pid = Sens_Process.pid
        comm_pid = Comm_Process.pid
        os.sched_setaffinity(sens_pid,set([0,1,2]))
        os.sched_setaffinity(comm_pid,set([3]))
       # S_Enable = True
        logger.info("Open")
        


if __name__ == "__main__":
    logger.info("POWER ON")
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(pin,GPIO.RISING,callback=button_callback, bouncetime=400)
    # connected=Connect_To_GUI()
    
    ### When add GUI add LED to demonstrate the internet connection status
    # if connected == True:
    #    print("GUI_CTRL_MODE")
    #    LED = ON
    # else:
    #    print("LOCAL_CTRL_MODE")
    #    LED = OFF
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        pass

#sample script
#python sensing.py --surveydistance 100.0 --wheelbase 1300 --heightthreashold 10.0 --actualboardwidth 13.6 --lasercolor green --gpu 0
