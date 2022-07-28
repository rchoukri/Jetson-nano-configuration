'''
https://github.com/abakisita/camera_calibration

This script is for generating data
1. Provide desired path to store images.
2. Press 'c' to capture image and display it.
3. Press any button to continue.
4. Press 'q' to quit.
'''

#from pydoc import visiblename
import cv2
import os
import numpy as np

def patron(img):
    nRows = 8
    nCols = 6 #5
    dimension = 600 #- mm
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, dimension, 0.001)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (nCols,nRows))
    if ret:
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        # Draw and display the corners
        cv2.drawChessboardCorners(img, (nCols,nRows), corners2,ret)
    print(ret)
    # cv2.imshow("Gray",img)


camera = cv2.VideoCapture(0)
ret, img = camera.read()

path='chess_image_calibration_flight1' # accede por rutas relativas

print(path)
count = 0
while True:
    if not os.path.isdir(path):
        print("No such a directory: {}".format(path))
        exit(1)

    name = path + str(count) +'.jpg'
    ret, img = camera.read()
    # im = client.simGetImage(vista, airsim.ImageType.Scene)
    # img = cv2.imdecode(airsim.string_to_uint8_array(im), cv2.IMREAD_UNCHANGED)

    patron(img)
    cv2.imshow("img", img)

    if cv2.waitKey(20) & 0xFF == ord('c'):
        print(name)
        cv2.imwrite(name, img)
        cv2.imshow("img", img)
        count += 1
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break;
