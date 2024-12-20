import os
import cv2
import numpy as np
import subprocess

def timer(func):
    def func_wrapper(*args, **kwargs):
        from time import time
        time_start = time()
        result = func(*args, **kwargs)
        time_end = time()
        time_spend = time_end - time_start
        print('%s cost time: %.3f s' % (func.__name__, time_spend))
        return result
    return func_wrapper


class Sensing():
    def __init__(self,actual_x=20,laser_color="red",gpu=0):
        self.cap = cv2.VideoCapture(0) #select camera
        self.actual_x = actual_x #length of target board from left to right
        self.on = 0 #Switch
        self.laser_color = laser_color #Color of Laser
        self.resolution = [1080,720]
        self.gpu = gpu
        self.cross = []
    def find_cross(self,image): #Find the center cross coordinate
        #image processing to find the cross line clearly
        gray_cross = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        side = cv2.Canny(gray_cross, 40, 100)
        kernel_erode = np.ones((3,3),np.uint8)
        
        dilate = cv2.dilate(side, kernel_erode, iterations=2)
        eroded = cv2.erode(dilate, kernel_erode, iterations=2)
        
        kernel1 = cv2.getStructuringElement(cv2.MORPH_RECT, (50, 1))
        kernel2 = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 50))
        kernel3 = cv2.getStructuringElement(cv2.MORPH_CROSS, (13, 13))
        
        hline = cv2.morphologyEx(eroded, cv2.MORPH_OPEN, kernel1)
        vline = cv2.morphologyEx(eroded, cv2.MORPH_OPEN, kernel2)
        
        
        contours, _ = cv2.findContours(hline, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        #create a blackboard to draw line detected
        zero = np.zeros(hline.shape, dtype=np.uint8)
        max_area = 0
        index = 0
        image_height, image_width, _ = image.shape
        #draw largest area horizontal line on blackboard
        for i in range(len(contours)):
            height = contours[i][:,0,1]
            max_h = np.max(height)
            min_h = np.min(height)
            ave_h = (max_h + min_h)/2
            if ave_h >= 1/5*image_height and ave_h <= 4/5*image_height:
                area = cv2.contourArea(contours[i])
                if area > max_area:
                    max_area = area
                    index = i
                    cv2.drawContours(zero, contours, index, (255, 255, 255), -1)
                
        contours1, _ = cv2.findContours(vline, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        max_area1 = 0
        index1 = 0
        #draw largest area vertical line on blackboard
        for i in range(len(contours1)):
            width = contours1[i][:,0,0]
            max_w = np.max(width)
            min_w = np.min(width)
            ave_w = (max_w + min_w)/2
            if ave_w >= 1/5*image_width and ave_w <= 4/5*image_width:
                area = cv2.contourArea(contours1[i])
                if area > max_area1:
                    max_area1 = area
                    index1 = i
                    cv2.drawContours(zero, contours1, index1, (255, 255, 255), -1)
        #Find the Center Points of Two Lines
        
        mask = cv2.morphologyEx(zero, cv2.MORPH_OPEN, kernel3)
        
        combine, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #Error handling
        if len(combine) == 0:
            cv2.circle(image, (int(self.cross[0]),int(self.cross[1])), 2, (0, 0, 255), -1)
            return self.cross[0]
        else:
            cross = cv2.minAreaRect(combine[0])
            center = (int(cross[0][0]), int(cross[0][1]))
            cv2.circle(image, center, 2, (0, 0, 255), -1)
            if self.cross == []:
                self.cross = [cross[0][0], cross[0][1]]
            return cross[0][0]
        
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
        normalized_image = cv2.merge([norm_b,norm_g,norm_r])
        return normalized_image
    
        #cite https://stackoverflow.com/questions/19181485/splitting-image-using-opencv-in-python
    def pt(self,image): #perspective transformation
        roi = np.array([[338,0],[76,0],[37,297],[374,297]])
        #      b,a,d,c
        #      a------b
        #      |      |
        #      d------c
        #measured by test_color.py
        #image processing to find the target board clearly
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        blur = cv2.GaussianBlur(gray,(5, 5),0)
        kernel = np.ones((7, 7), np.uint8)
        closed = cv2.morphologyEx(blur, cv2.MORPH_CLOSE, kernel)
        canny = cv2.Canny(closed, 35, 150)

        rectangle = np.array(0)
        #Find all the lines in the photo
        lines = cv2.HoughLinesP(canny,1,np.pi/180, threshold = 60, minLineLength = 200, maxLineGap = 45)

        vertical_lines = []
        height, width = image.shape[:2]
        
        #find slope of lines to find vertical line
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else np.inf
                if abs(slope) > 0.8:
                    vertical_lines.append((x1, y1, x2, y2))

        if len(vertical_lines) != 2:
            rectangle=roi
        else:
            intersection_points = []
        #calculate the cross points of vertical line and edge of image
            for line in vertical_lines:
                x1, y1, x2, y2 = line
                slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else np.inf
                intercept = y1 - slope * x1 if slope != np.inf else 0
                y_top = 0
                if slope != np.inf:
                    x_top = (y_top - intercept) / slope
                    intersection_points.append((int(x_top), int(y_top)))
                else:
                    intersection_points.append((int(x1), int(y_top)))
                y_bottom = height - 1
                if slope != np.inf:
                    x_bottom = (y_bottom - intercept) / slope
                    intersection_points.append((int(x_bottom), int(y_bottom)))
                else:
                    intersection_points.append((int(x1), int(y_bottom)))
            rectangle=np.array(intersection_points).reshape(4,2)
            if abs(np.min(rectangle[rectangle[:, 1] > np.mean(rectangle[:, 1])]) - np.max(rectangle[rectangle[:, 1] < np.mean(rectangle[:, 1])])) < 250: #Error handling If the distance between two side is too small.
                rectangle=roi
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
        maxWidth = max(int(width1), int(width2))
        
        hight1 = np.sqrt(((b[0] - c[0])**2) + ((b[1] - c[1])**2))
        hight2 = np.sqrt(((a[0] - d[0])**2) + ((a[1] - d[1])**2))
        maxHeight = max(int(hight1), int(hight2))
        #Transform the target board to front perspective, 
        transformed = np.array([[0, 0],[maxWidth - 1, 0],[maxWidth - 1, maxHeight - 1],[0, maxHeight - 1]],dtype=np.float32)
        board_x = maxWidth - 1
        matrix = cv2.getPerspectiveTransform(lazer_board, transformed)
        image = cv2.warpPerspective(image,matrix,(maxWidth,maxHeight))
        image = self.adjust_lightness(image)
        return image, board_x

    def find_circle(self,image): #Find the center of circle/eclipse/half circle created by red laser
        #image processing to find the laser dot clearly
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        if self.laser_color == "red":
            lower_color = np.array([160, 20, 200])
            upper_color = np.array([180, 120, 255])
        elif self.laser_color == "green":
            lower_color = np.array([68, 27, 240])
            upper_color = np.array([90, 80, 255])
        else:
            pass
        kernel = np.ones((5,5),np.uint8)
        kernel_dilate = np.ones((5,5),np.uint8)
        
        mask = cv2.inRange(hsv, lower_color, upper_color)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        dilate = cv2.dilate(mask, kernel_dilate, iterations=3)
        
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
            if cv2.arcLength(contour, True) == 0 or cv2.contourArea(contour) < 100:
                continue
            else:
                circularity = (4 * np.pi * cv2.contourArea(contour)) / (cv2.arcLength(contour, True) ** 2)
            if circularity >= 0.7:
                cv2.drawContours(mask, [contour], -1, (255), thickness=cv2.FILLED)
                cv2.circle(image, (int(x), int(y)), 2, (0, 0, 255), -1)
                target_x = x
                break
            elif 0.4 < circularity < 0.7:
                if np.any(contour[:, 0, 1] == 0) or np.any(contour[:, 0 ,1] == image.shape[0]-1) or (np.any(contour[:,0,0] == 0) or np.any(contour[:,0,0] == image.shape[1]-1)):
                    cv2.drawContours(mask, [contour], -1, (255), thickness=cv2.FILLED)
                    cv2.circle(image, (int(x), int(y)), 2, (0, 0, 255), -1)
                    target_x = x
                    break
        return target_x
    
    @timer
    def off_dis(self,img): #calculate the distance between laser dot and center cross
        img, board_x=self.pt(img)
        target_x = self.find_circle(img)
        target_cross = self.find_cross(img)
        cv2.imshow('Camera', img)
        if target_x == 0:
            return "No Circle Detected"
        else:
            dis_off = (target_x - target_cross)/board_x*self.actual_x
            return dis_off
    
    def start_sensing(self): #final api for use (not functional now)
        while self.on == 1:
            ret , frame = self.cap.read()
            if not ret:
                print("Camera Connect Failed")
                break
            
            dis_off = self.off_dis(frame)
            #self.communication(dis_off)
            #dis_off send to controller
            
            #if condition met for close the machine
            #self.on = 0
            
        self.cal.release()
        cv2.destroyAllWindows()
        
    def test(self):
        #cap = cv2.VideoCapture(0)
        self.cap.set(3, self.resolution[0])
        self.cap.set(4, self.resolution[1])
        if not self.cap.isOpened():
            print("Error: Could not open video file.")
            exit()
        
        while self.cap.isOpened():
            ret, frame = self.cap.read()
        
            if not ret:
                break
            
            # Display the frame
            new_width = int(frame.shape[1] * 0.5)
            new_height = int(frame.shape[0] * 0.5)
            frame = cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
            
            dis_off = self.off_dis(frame)
            if dis_off == "error":
                print("Error detecting the Laser Board")
            else:
                print(dis_off)
                
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()
    
    def communication(self,value):
        #Create Communication Protocol with STM32
        pass

if __name__ == "__main__":
    dis = Sensing()
    #dis.test("D:/capstone/capstone/sensing/test_video/test.mp4")
    dis.test()