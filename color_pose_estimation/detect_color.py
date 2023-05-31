import cv2 as cv2
import os
import numpy as np
import imutils


def detect(img):

    # hsv = hue, saturation, value
    hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # specify blue range that should be detected
    low_blue = np.array([173, 100, 100])
    high_blue = np.array([190, 255, 255])

    blue_mask = cv2.inRange(hsv_frame, low_blue, high_blue)
    blue = cv2.bitwise_and(img, img, mask=blue_mask)

    #cv2.imshow("Blue", blue)
    #cv2.imshow("mask", blue_mask)

    masked_img = blue_mask.astype(np.uint8)

    # find contours in the thresholded image
    cnts = cv2.findContours(
        masked_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    print("[INFO] {} unique contours found".format(len(cnts)))
    l_w_h = 0
    l_rect = 0 , 0 , 0 , 0

    # sometimes small particles are detected, too - filter the largest rectangle
    for contour in cnts:
        rect = cv2.boundingRect(contour)
        x, y, w, h = rect
        if w*h > l_w_h:
            l_w_h = w*h
            l_rect = rect
    x, y, w, h = l_rect

    cv2.rectangle(img, (x,y),(x+w, y+h), (255,0,0), 2)
    cv2.imshow("mask", img)

    key = cv2.waitKey(1)

    return l_rect
