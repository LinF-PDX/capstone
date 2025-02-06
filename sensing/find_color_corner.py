#Due to unstable functionality of finding the edge of target board use this for aid
from backend import Sensing
import cv2
import time
import numpy as np

#video_path = 'D:/capstone/capstone/sensing/test_video/test.mp4'

#image_path = 'D:/capstone/test21.jpg'
#img = cv2.imread(image_path)


def config_img():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_SETTINGS, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS,60)
    cap.set(cv2.CAP_PROP_BRIGHTNESS,136)
    cap.set(cv2.CAP_PROP_CONTRAST,35)
    cap.set(cv2.CAP_PROP_SATURATION,40)
    cap.set(cv2.CAP_PROP_HUE,-600)
    cap.set(cv2.CAP_PROP_GAMMA,140)
    cap.set(cv2.CAP_PROP_AUTOFOCUS,0)
    cap.set(cv2.CAP_PROP_FOCUS,255)
    cap.set(cv2.CAP_PROP_AUTO_WB,0)
    cap.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 4700)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.release()
def get_img():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS,60)
    cap.set(cv2.CAP_PROP_BRIGHTNESS,136)
    cap.set(cv2.CAP_PROP_CONTRAST,35)
    cap.set(cv2.CAP_PROP_SATURATION,40)
    cap.set(cv2.CAP_PROP_HUE,-600)
    cap.set(cv2.CAP_PROP_GAMMA,140)
    cap.set(cv2.CAP_PROP_AUTOFOCUS,0)
    cap.set(cv2.CAP_PROP_FOCUS,255)
    cap.set(cv2.CAP_PROP_AUTO_WB,0)
    cap.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 4700)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.release()
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    time.sleep(3)
    ret, img = cap.read()
    cap.release()
    return img
def load_img():
    #image_path = 'D:/capstone/capstone/sensing/error/image_300.jpg'
    image_path = 'D:/capstone/capstone/sensing/whiteyellow-wb-far/test53.jpg'
    img = cv2.imread(image_path)
    return img

sensing = Sensing()
img = load_img()
image , _ = sensing.pt(img)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

points = []

#This function to find the color range for laser
def find_color(event, x, y, flags, para):
    if event == cv2.EVENT_LBUTTONDOWN:
        print('PIX:', x, y)
        print("HSV:", hsv[y, x])
#This function to find the edge of target board
def find_corner(event, x, y, flags, param):
    #follow upper right, upper left, lower left, lower right and replace roi in sensing.py
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        print(f"Point {len(points)}: ({x}, {y})")
        cv2.circle(img, (x, y), 5, (0, 255, 0), -1)

if __name__ == '__main__':
    cv2.namedWindow("frame")
    cv2.setMouseCallback("frame", find_corner)
    while True:
        cv2.imshow('frame', img)        
        if cv2.waitKey() == ord('q'):
            break
    cv2.destroyAllWindows()

