#Due to unstable functionality of finding the edge of target board use this for aid

import cv2

#video_path = 'D:/capstone/capstone/sensing/test_video/test.mp4'
cap = cv2.VideoCapture(0)
#image_path = 'D:/capstone/test21.jpg'
#img = cv2.imread(image_path)
cap.set(3, 1080)
cap.set(4, 720)
ret, img = cap.read()
cap.release()
new_width = int(img.shape[1] * 0.5)
new_height = int(img.shape[0] * 0.5)

img = cv2.resize(img, (new_width, new_height), interpolation=cv2.INTER_LINEAR)

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
    cv2.namedWindow("img")
    cv2.setMouseCallback("img", find_corner)
    while True:
        cv2.imshow('img', img)
        if cv2.waitKey() == ord('q'):
            break
    cv2.destroyAllWindows()
