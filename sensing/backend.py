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
    os.system('sudo ip link set can1 type can bitrate 100000')
    os.system('sudo ifconfig can1 up')
    can1 = can.interface.Bus(channel='can1', interface='socketcan')
    return can0, can1


def can_send(data,can0):
    data_send = struct.pack('<b', data)
    msg = can.Message(is_extended_id=False, arbitration_id=0x123, data=data_send)
    can0.send(msg)

def can_receive(can1):
    msg = can1.recv(0.01)  # receive(time out)
    if msg is None:
        return None, None
        #print('Timeout, no msg')
    else:
        data= struct.unpack('<dd', msg)
        return data[0],data[1]

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
        self.cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
        self.ActualBoardWidth = ActualBoardWidth #width of target board
        self.laser_color = laser_color
        self.S_Resolution = [640,480]
        self.gpu = gpu
        self.cross = []
        self.roi = np.array([[485,76],[174,83],[151,357],[517,355]])
         
        #      b,a,d,c
        #      a------b
        #      |      |
        #      d------c
        #measured by find_color_corner.py
        #image processing to find the target board clearly
    def find_cross(self,image):
        #image processing to find the cross line clearly
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        side = cv2.Canny(gray, 40, 100)
        kernel_erode = np.ones((3,3),np.uint8)
        dilate = cv2.dilate(side, kernel_erode, iterations=2)
        eroded = cv2.erode(dilate, kernel_erode, iterations=2)
        kernel1 = cv2.getStructuringElement(cv2.MORPH_RECT, (50, 1))
        kernel2 = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 50))
        kernel3 = cv2.getStructuringElement(cv2.MORPH_CROSS, (13, 13))
        
        horizon = cv2.morphologyEx(eroded, cv2.MORPH_OPEN, kernel1)
        verticle = cv2.morphologyEx(eroded, cv2.MORPH_OPEN, kernel2)
        contours, _ = cv2.findContours(horizon, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        #contours = sorted(contours, key=cv2.contourArea, reverse=True)
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
        
        contours1, _ = cv2.findContours(verticle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
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
        mask = cv2.morphologyEx(zero, cv2.MORPH_OPEN, kernel3)
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
        
    def adjust_lightness(self,image):
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
    
        #cite https://stackoverflow.com/questions/19181485/splitting-image-using-opencv-in-python
    
    def pt(self,image): #perspective transformation
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # blur = cv2.GaussianBlur(gray,(5, 5),0)
        # kernel = np.ones((7, 7), np.uint8)
        # opening = cv2.morphologyEx(blur, cv2.MORPH_CLOSE, kernel)
        # canny = cv2.Canny(opening, 35, 150)
        #rectangle = np.array(0)
        rectangle = self.roi
        #Find all the lines in the photo
        # lines = cv2.HoughLinesP(canny,1,1*np.pi/180, threshold = 60, minLineLength = 200, maxLineGap = 45)
        # ver_l = []
        # lslope = []
        # height, width = image.shape[:2]
        #find slope of lines to find vertical line
        # if lines is not None:
        #     for line in lines:
        #         x1, y1, x2, y2 = line[0]
        #         if (x2 - x1) != 0:
        #             slope = (y2 - y1) / (x2 - x1)
        #         else:
        #             slope = np.inf
        #         if abs(slope) > 0.8:
        #             ver_l.append((x1, y1, x2, y2))
        #             lslope.append(slope)

        # if len(ver_l) != 2:
        #     rectangle=self.roi
        # else:
        #     intersect = []
        # #calculate the cross points of vertical line and edge of image
        #     for line, slope in zip(ver_l, lslope):
        #         x1, y1, x2, y2 = line
        #         if slope != np.inf:
        #             intercept = y1 - slope * x1
        #         else:
        #             intercept = 0
        #         y_top = 0
        #         y_bottom = height - 1
        #         if slope != np.inf:
        #             x_top = (y_top - intercept) / slope
        #             intersect.append((int(x_top), int(y_top)))
        #             x_bottom = (y_bottom - intercept) / slope
        #             intersect.append((int(x_bottom), int(y_bottom)))
        #         else:
        #             intersect.append((int(x1), int(y_top)))
        #             intersect.append((int(x1), int(y_bottom)))
        #     rectangle=np.array(intersect).reshape(4,2)
        #     if abs(np.min(rectangle[rectangle[:, 1] > np.mean(rectangle[:, 1])]) - np.max(rectangle[rectangle[:, 1] < np.mean(rectangle[:, 1])])) < 200: #If the distance between two side is too small.
        #         rectangle=self.roi
        #     else:
        #        self.roi = rectangle
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
        image = self.adjust_lightness(image)
        return image, board_x

    def find_circle(self,image): #Find the center of circle/eclipse/half circle created by green laser
        #image processing to find the laser dot clearly
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        if self.laser_color == "red":
            low = np.array([160, 20, 200])
            high = np.array([180, 120, 255])
        elif self.laser_color == "green":
            #low = np.array([68, 27, 240])
            #high = np.array([90, 80, 255])
            low = np.array([40, 20, 210])
            high = np.array([90, 80, 210])
        else:
            pass
        kernel = np.ones((5,5),np.uint8)
        kernel_dilate = np.ones((5,5),np.uint8)
        mask = cv2.inRange(hsv, low, high)
        opening = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        dilate = cv2.dilate(opening, kernel_dilate, iterations=3)
        blur = cv2.GaussianBlur(dilate, (7, 7), 0)
        contours, _ = cv2.findContours(blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas = [cv2.contourArea(cnt) for cnt in contours]
        if len(areas) == 0:
            sorted_contours = contours
        else:
            _ , sorted_contours = zip(*sorted(zip(areas, contours), key=lambda x: x[0], reverse=True))
        mask = np.zeros_like(mask)
        target_x = 0
        #Using similarity of the laser dot to a circle to find the center point
        for contour in sorted_contours:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            if cv2.arcLength(contour, True) == 0 or cv2.contourArea(contour) < 500 and self.laser_color == "red":
                continue
            elif cv2.arcLength(contour, True) == 0 or cv2.contourArea(contour) < 500 and self.laser_color == "green":
                continue
            else:
                circularity = (4 * np.pi * cv2.contourArea(contour)) / (cv2.arcLength(contour, True) ** 2)
            if circularity >= 0.75:
                cv2.drawContours(mask, [contour], -1, (255), thickness=cv2.FILLED)
                cv2.circle(image, (int(x), int(y)), 2, (0, 0, 255), -1)
                target_x = x
                break
            elif 0.55 < circularity < 0.75 and (np.any(contour[:, 0, 1] == 0) or np.any(contour[:, 0 ,1] == image.shape[0]-1) or (np.any(contour[:,0,0] == 0) or np.any(contour[:,0,0] == image.shape[1]-1))):
                 # circle in the corner
                    cv2.drawContours(mask, [contour], -1, (255), thickness=cv2.FILLED)
                    cv2.circle(image, (int(x), int(y)), 2, (0, 0, 255), -1)
                    target_x = x
                    break
        return target_x
    
    @timer
    def off_dis(self,img): #calculate the distance between laser dot and center cross
        img, board_x=self.pt(img)
        target_cross = self.find_cross(img)
        if target_cross == "error":
            return "error"
        target_x = self.find_circle(img)
        #cv2.imshow('Camera', img)
        if target_x == 0:
            return 100
        else:
            dis_off = (target_x - target_cross)/board_x*self.ActualBoardWidth
            return dis_off
        
    def camera_setup(self):
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
        self.cap.set(cv2.CAP_PROP_AUTO_WB,3)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,1)
        self.cap.release()
        self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        
    def test(self):
        self.camera_setup()
        if not self.cap.isOpened():
            print("Error: Could not open video file.")
            exit()
        
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break
            
            # Display the frame
            #new_width = int(frame.shape[1] * 0.5)
            #new_height = int(frame.shape[0] * 0.5)
            #frame = cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
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
