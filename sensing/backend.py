import cv2
import numpy as np
import os
import can
import struct

def add_parser_arguments(parser):
    parser.add_argument("--surveydistance", type=float, default=100.0,
        help="S_surveyDistance")
    parser.add_argument("--wheelbase", type=int, default=1300,
        help="S_wheelBase")
    parser.add_argument("--heightthreashold", type=float, default=10.0,
        help="S_heightThreashold")
    parser.add_argument("--actualboardwidth", type=float, default=13.6,
        help="Actual Width of laser board in cm")
    parser.add_argument("--lasercolor", type=str, default="green",
        help="Color of laser gun used for tracking")
    parser.add_argument("--gpu", type=bool, default=0,
        help="Enable GPU for Opencv (Not Available Now)")

def timer(func):
    def func_wrapper(*args, **kwargs):
        from time import time
        t1 = time()
        result = func(*args, **kwargs)
        time_spend = time() - t1
        print('%s cost time: %f s' % (func.__name__, time_spend))
        return result
    return func_wrapper

def can_init():
    os.system('sudo ip link set can0 type can bitrate 500000')
    os.system('sudo ifconfig can0 up')
    can0 = can.interface.Bus(channel='can0', interface='socketcan')
    os.system('sudo ip link set can1 type can bitrate 500000')
    os.system('sudo ifconfig can1 up')
    can1 = can.interface.Bus(channel='can1', interface='socketcan')
    return can0, can1


def can_send(data,can0):
    data_send = struct.pack('<b', data)
    msg = can.Message(is_extended_id=False, arbitration_id=0x123, data=data_send)
    can0.send(msg)

def can_receive(can0):
    msg = can0.recv(0.01)  # receive(time out)
    if msg is None:
        return None, None
        #print('Timeout, no msg')
    else:
        data= struct.unpack('<Hh', msg.data[:4])
        return float(data[0])/100,float(data[1])/10

def can_initvalue(args,S_Enable,can0):
    S_surveydistance = args.surveydistance
    S_wheelbase = args.wheelbase
    S_heightthreashold = args.heightthreashold
    data_tuple=(S_surveydistance,S_wheelbase,S_heightthreashold,S_Enable)
    data_init=struct.pack('<BHb?',data_tuple)
    msg=can.Message(is_extended_id = False, arbitration_id=0x102,data=data_init)
    can0.send(msg)

def can_down():
    os.system('sudo ifconfig can0 down')
    os.system('sudo ifconfig can1 down')


class Sensing():
    def __init__(self,ActualBoardWidth=13.6,laser_color="green",gpu=0):
        self.cap = None
        self.ActualBoardWidth = ActualBoardWidth #width of target board
        self.laser_color = laser_color
        self.S_Resolution = [640,480]
        self.gpu = gpu
        self.cross = []
        self.roi = np.array([[536,64],[98,61],[60,479],[580,479]])
        #self.roi = np.array([[522,16],[93,35],[63,452],[569,443]])
        #      b,a,d,c
        #      a------b
        #      |      |
        #      d------c
        #measured by find_color_corner.py
        #image processing to find the target board clearly
    def find_cross(self,image):
        #image processing to find the cross line clearly
        image = self.remove_shadow(image)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        side = cv2.Canny(gray, 40, 100)
        kernel_erode = np.ones((3,3),np.uint8)
        dilate = cv2.dilate(side, kernel_erode, iterations=2)
        eroded = cv2.erode(dilate, kernel_erode, iterations=2)
        kernel_horizontal = cv2.getStructuringElement(cv2.MORPH_RECT, (50, 1))
        kernel_vertical = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 50))
        kernel_cross = cv2.getStructuringElement(cv2.MORPH_CROSS, (13, 13))
        
        horizon = cv2.morphologyEx(eroded, cv2.MORPH_OPEN, kernel_horizontal)
        vertical = cv2.morphologyEx(eroded, cv2.MORPH_OPEN, kernel_vertical)
        contours, _ = cv2.findContours(horizon, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        #create a blackboard to draw line detected
        zero = np.zeros(horizon.shape, dtype=np.uint8)
        max_area = 0
        index = 0
        image_height, image_width, _ = image.shape
        #draw largest area horizontal line on blackboard
        for i in range(len(contours)):
            height = contours[i][:,0,1]
            max_h = np.max(height)
            min_h = np.min(height)
            ave_h = (max_h + min_h)/2
            if ave_h >= 1/3*image_height and ave_h <= 2/3*image_height:
                area = cv2.contourArea(contours[i])
                if area > max_area:
                    max_area = area
                    index = i
        cv2.drawContours(zero, contours, index, (255, 255, 255), -1)
        
        contours1, _ = cv2.findContours(vertical, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        max_area1 = 0
        index1 = 0
        #draw largest area vertical line on blackboard
        for i in range(len(contours1)):
            width = contours1[i][:,0,0]
            max_w = np.max(width)
            min_w = np.min(width)
            ave_w = (max_w + min_w)/2
            if ave_w >= 1/3*image_width and ave_w <= 2/3*image_width:
                area = cv2.contourArea(contours1[i])
                if area > max_area1:
                    max_area1 = area
                    index1 = i
        cv2.drawContours(zero, contours1, index1, (255, 255, 255), -1)
        #Find the Center Points of Two Lines
        mask = cv2.morphologyEx(zero, cv2.MORPH_OPEN, kernel_cross)
        combine, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(combine) == 0:
            if self.cross == []:
                return "error"
            cv2.circle(image, (int(self.cross[0]),int(self.cross[1])), 2, (0, 0, 255), -1)
            return self.cross[0]
        else:
            for square in combine:
                cross = cv2.minAreaRect(square)
                if cross[0][0] >= 1/3*image_width and cross[0][0] <= 2/3*image_width:
                    if cross[0][1] >= 1/3*image_height and cross[0][1] <= 2/3*image_height:
                        center = (int(cross[0][0]), int(cross[0][1]))
                        cv2.circle(image, center, 2, (0, 0, 255), -1)
                        if self.cross == []:
                            self.cross = [cross[0][0], cross[0][1]]
                        return cross[0][0]
            if self.cross == []:
                return "error"
            cv2.circle(image, (int(self.cross[0]),int(self.cross[1])), 2, (0, 0, 255), -1)
            return self.cross[0]
        
    def remove_shadow(self,image):
        b,g,r = cv2.split(image)
        b_d = cv2.dilate(b, np.ones((11,11), np.uint8))
        blur_b = cv2.medianBlur(b_d, 21)
        diff_b = 255 - cv2.absdiff(b, blur_b)
        norm_b = cv2.normalize(diff_b, None, alpha=100, beta=210, norm_type=cv2.NORM_MINMAX)
        g_d = cv2.dilate(g, np.ones((11,11), np.uint8))
        blur_g = cv2.medianBlur(g_d, 21)
        diff_g = 255 - cv2.absdiff(g, blur_g)
        norm_g = cv2.normalize(diff_g, None, alpha=100, beta=210, norm_type=cv2.NORM_MINMAX)
        r_d = cv2.dilate(r, np.ones((11,11), np.uint8))
        blur_r = cv2.medianBlur(r_d, 21)
        diff_r = 255 - cv2.absdiff(r, blur_r)
        norm_r = cv2.normalize(diff_r, None, alpha=100, beta=210, norm_type=cv2.NORM_MINMAX)
        norm_img = cv2.merge([norm_b,norm_g,norm_r])
        return norm_img
        #cite https://stackoverflow.com/questions/44752240/how-to-remove-shadow-from-scanned-images-using-opencv
    
    def pt(self,image): #perspective transformation
        rectangle = self.roi
        #Using four dots as the corner of target board
        board_x = 0
        x_mean = np.mean(rectangle[:, 0])
        y_mean = np.mean(rectangle[:, 1])
        lazer_board = np.zeros((4,2),dtype=np.float32)
        lazer_board[0] = rectangle[(rectangle[:, 0] < x_mean) & (rectangle[:, 1] < y_mean)] #upper left corner
        lazer_board[1] = rectangle[(rectangle[:, 0] > x_mean) & (rectangle[:, 1] < y_mean)] #upper right corner
        lazer_board[2] = rectangle[(rectangle[:, 0] > x_mean) & (rectangle[:, 1] > y_mean)] #lower right corner
        lazer_board[3] = rectangle[(rectangle[:, 0] < x_mean) & (rectangle[:, 1] > y_mean)] #lower left corner
        a = lazer_board[0]
        b = lazer_board[1]
        c = lazer_board[2]
        d = lazer_board[3]
        
        #      a------b
        #      |      |
        #      d------c
        
        width1 = np.sqrt(((c[0] - d[0])**2) + ((c[1] - d[1])**2))
        width2 = np.sqrt(((b[0] - a[0])**2) + ((b[1] - a[1])**2))
        maxW = max(int(width1), int(width2))
        
        hight1 = np.sqrt(((b[0] - c[0])**2) + ((b[1] - c[1])**2))
        hight2 = np.sqrt(((a[0] - d[0])**2) + ((a[1] - d[1])**2))
        maxH = max(int(hight1), int(hight2))
        
        #Transform the target board to front perspective, 
        transformed = np.array([[0, 0],[maxW - 1, 0],[maxW - 1, maxH - 1],[0, maxH - 1]],dtype=np.float32)
        board_x = maxW - 1
        matrix = cv2.getPerspectiveTransform(lazer_board, transformed)
        image = cv2.warpPerspective(image,matrix,(maxW,maxH))
        return image, board_x
    
    def detect_laser_blob(self, image):
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = 80 
        params.maxArea = 5000
        params.minThreshold = 160
        params.maxThreshold = 240 
        params.thresholdStep = 10
        params.minRepeatability = 2
        params.filterByColor = False 
        params.filterByInertia = True
        params.minInertiaRatio = 0.1
        params.filterByConvexity = True
        params.minConvexity = 0.8
        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(image)
        target_x = "error"
        target_y = "error"
        if keypoints:
            blob = max(keypoints, key=lambda k: k.size)
            target_x, target_y = blob.pt
            #cv2.circle(image, (int(target_x), int(target_y)), 2, (0, 0, 255), -1)
            return target_x, target_y
        else:
            return target_x, target_y
        
    def detect_laser_color(self,image): #Find the center of circle/eclipse/half circle created by green laser
        #image processing to find the laser dot clearly
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        if self.laser_color == "red":
            lower_hsv = np.array([160, 20, 200])
            upper_hsv = np.array([180, 120, 255])
            mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        elif self.laser_color == "green":
            lower_hsv_ll = np.array([25, 1, 245])  
            upper_hsv_ll = np.array([100, 70, 255])
            lower_hsv_hl = np.array([35, 80, 240]) 
            upper_hsv_hl = np.array([95, 255, 255])
            mask_ll = cv2.inRange(hsv, lower_hsv_ll, upper_hsv_ll)
            mask_hl = cv2.inRange(hsv, lower_hsv_hl, upper_hsv_hl)
            mask = cv2.bitwise_or(mask_ll, mask_hl)
        else:
            return "error"
        close = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        opens = cv2.morphologyEx(close, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        blur = cv2.GaussianBlur(opens, (5, 5), 0)
        dialate = cv2.dilate(blur, np.ones((3, 3), np.uint8))        
        contours, _ = cv2.findContours(dialate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas = [cv2.contourArea(cnt) for cnt in contours]
        if len(areas) == 0:
            sorted_contours = contours
        else:
            _ , sorted_contours = zip(*sorted(zip(areas, contours), key=lambda x: x[0], reverse=True))
        target_x = "error"
        target_y = "error"
        #Using similarity of the laser dot to a circle to find the center point
        for contour in sorted_contours:
            #If future testing shows that the laser dot is not a circle, change the following code to fitEllipse
            #ellipse = cv2.fitEllipse(contours)
            #center = ellipse[0]
            area = cv2.contourArea(contour)
            arclength = cv2.arcLength(contour, True)
            if cv2.arcLength(contour, True) == 0 or area < 100 or area > 5000:
                continue
            (x, y), radius = cv2.minEnclosingCircle(contour)
            circularity = (4 * np.pi * area) / (arclength ** 2)
            
            if circularity >= 0.75:
                #cv2.circle(image, (int(x), int(y)), 2, (0, 0, 255), -1)
                target_x = x
                target_y = y
                break
            elif 0.55 < circularity < 0.75 and (np.any(contour[:, 0, 1] == 0) or np.any(contour[:, 0 ,1] == image.shape[0]-1) or (np.any(contour[:,0,0] == 0) or np.any(contour[:,0,0] == image.shape[1]-1))): # circle in the corner
                #cv2.circle(image, (int(x), int(y)), 2, (0, 0, 255), -1)
                target_x = x
                target_y = y
                break
            else:
                pass
        return target_x, target_y
    
    def detect_laser_cnn(self, image):
        return 0
    
    def find_circle(self, image):
        target_x_blob, target_y_blob = self.detect_laser_blob(image)
        target_x_color, target_y_color = self.detect_laser_color(image)
        
        if target_x_blob ==  "error" and target_x_color == "error":
            return "error"
        elif target_x_blob == "error" and target_x_color != "error":
            return target_x_color
        elif target_x_blob != "error" and target_x_color == "error":
            return target_x_blob
        else:
            distance = np.sqrt((target_x_blob - target_x_color)**2 + (target_y_blob - target_y_color)**2)
            if distance < 10:
                return target_x_color
            else:
                value=self.detect_laser_cnn(image)
                return "error"
    @timer
    def off_dis(self,img): #calculate the distance between laser dot and center cross
        img, board_x=self.pt(img)
        target_cross = self.find_cross(img)
        if target_cross == "error":
            return "error"
        target_x = self.find_circle(img)
        #cv2.imshow('Camera', img)
        if target_x == "error":
            return 100
        else:
            dis_off = (target_x - target_cross)/board_x*self.ActualBoardWidth
            return dis_off
        
    def camera_setup(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.S_Resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.S_Resolution[1])
        self.cap.set(cv2.CAP_PROP_FPS,60)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS,136)
        self.cap.set(cv2.CAP_PROP_CONTRAST,35)
        self.cap.set(cv2.CAP_PROP_SATURATION,40)
        self.cap.set(cv2.CAP_PROP_HUE,-600)
        self.cap.set(cv2.CAP_PROP_GAMMA,140)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS,0)
        self.cap.set(cv2.CAP_PROP_FOCUS,255)
        self.cap.set(cv2.CAP_PROP_AUTO_WB,0)
        self.cap.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 4700)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,1)
        self.cap.release()
        self.cap = cv2.VideoCapture(0)
    
    def test(self):
        self.camera_setup()
        if not self.cap.isOpened():
            print("Error: Could not open video file.")
            exit()
        frame = np.empty((480,640,3),dtype=np.uint8)
        count = 0
        while self.cap.isOpened():
            ret, _ = self.cap.read(frame)
            if not ret:
                break
            dis_off = self.off_dis(frame)
            if dis_off == "error":
                print("Error detecting the cross")
            else:
                print(dis_off)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
        self.cap.release()
        cv2.destroyAllWindows()
    
# if __name__ == "__main__":
#     import argparse
#     parser = argparse.ArgumentParser()
#     add_parser_arguments(parser)
#     args = parser.parse_args()
#     dis = Sensing(ActualBoardWidth=args.actualboardwidth,laser_color=args.lasercolor,gpu=args.gpu)
#     dis.test()
    #python backend.py --surveydistance 100.0 --wheelbase 1300 --heightthreashold 10.0 --actualboardwidth 13.6 --lasercolor green --gpu 0
