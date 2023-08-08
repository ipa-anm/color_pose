import cv2 as cv2
import os
import numpy as np
import imutils


def detect(img):

    # hsv = hue, saturation, value
    hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # specify yellow range that should be detected
    low_yellow = np.array([20, 30, 120])
    high_yellow = np.array([35, 255, 255])
    yellow_mask = cv2.inRange(hsv_frame, low_yellow, high_yellow)
    yellow = cv2.bitwise_and(img, img, mask=yellow_mask)
    masked_img_yellow = yellow_mask.astype(np.uint8)

    # specify blue range that should be detected
    low_blue = np.array([90, 150, 5])
    high_blue = np.array([130, 255, 255])
    blue_mask = cv2.inRange(hsv_frame, low_blue, high_blue)
    blue = cv2.bitwise_and(img, img, mask=blue_mask)

    # cv2.imshow("Blue", blue)
    # cv2.imshow("mask", blue_mask)

    masked_img_blue = blue_mask.astype(np.uint8)

    # specify red range that should be detected
    low_red = np.array([135, 30, 100])
    high_red = np.array([180, 255, 255])
    red_mask = cv2.inRange(hsv_frame, low_red, high_red)
    red = cv2.bitwise_and(img, img, mask=red_mask)
    masked_img_red = red_mask.astype(np.uint8)

    # specify green range that should be detected
    low_green = np.array([36, 30, 100])
    high_green = np.array([82, 255, 255])
    green_mask = cv2.inRange(hsv_frame, low_green, high_green)
    green = cv2.bitwise_and(img, img, mask=green_mask)
    masked_img_green = green_mask.astype(np.uint8)

    # find contours in the thresholded image
    cnts_green = cv2.findContours(
        masked_img_green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts_green = imutils.grab_contours(cnts_green)

    # find contours in the thresholded image
    cnts_red = cv2.findContours(
        masked_img_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts_red = imutils.grab_contours(cnts_red)

    # find contours in the thresholded image
    cnts_yellow = cv2.findContours(
        masked_img_yellow.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts_yellow = imutils.grab_contours(cnts_yellow)

    # find contours in the thresholded image
    cnts_blue = cv2.findContours(
        masked_img_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts_blue = imutils.grab_contours(cnts_blue)

    # print("[INFO] {} unique contours found".format(len(cnts)))
    l_w_h = 0
    l_rect_blue = 0, 0, 0, 0
    l_rect_red = 0, 0, 0, 0
    l_rect_yellow = 0, 0, 0, 0
    l_rect_green = 0, 0, 0, 0

    # sometimes small particles are detected, too - filter the largest rectangle
    for contour in cnts_green:
        rect = cv2.boundingRect(contour)
        x, y, w, h = rect
        if w*h > l_w_h:
            l_w_h = w*h
            l_rect_green = rect
    x, y, w, h = l_rect_green
    for contour in cnts_blue:
        rect = cv2.boundingRect(contour)
        x, y, w, h = rect
        if w*h > l_w_h:
            l_w_h = w*h
            l_rect_blue = rect
    x, y, w, h = l_rect_blue
    for contour in cnts_red:
        rect = cv2.boundingRect(contour)
        x, y, w, h = rect
        if w*h > l_w_h:
            l_w_h = w*h
            l_rect_red = rect
    x, y, w, h = l_rect_red
    for contour in cnts_yellow:
        rect = cv2.boundingRect(contour)
        x, y, w, h = rect
        if w*h > l_w_h:
            l_w_h = w*h
            l_rect_yellow = rect
    x, y, w, h = l_rect_yellow
    key = cv2.waitKey(1)


    if l_rect_yellow[2]*l_rect_yellow[3] > l_rect_blue[2]*l_rect_blue[3] and l_rect_yellow[2]*l_rect_yellow[3] > l_rect_green[2]*l_rect_green[3] and l_rect_yellow[2]*l_rect_yellow[3] > l_rect_red[2]*l_rect_red[3]:
        print("Found Yellow Cube")
        return l_rect_yellow
    if l_rect_red[2]*l_rect_red[3] > l_rect_blue[2]*l_rect_blue[3] and l_rect_red[2]*l_rect_red[3] > l_rect_green[2]*l_rect_green[3] and l_rect_red[2]*l_rect_red[3] > l_rect_yellow[2]*l_rect_yellow[3]:
        print("Found Red Cube")
        return l_rect_red
    if l_rect_blue[2]*l_rect_blue[3] > l_rect_red[2]*l_rect_red[3] and l_rect_blue[2]*l_rect_blue[3] > l_rect_green[2]*l_rect_green[3] and l_rect_blue[2]*l_rect_blue[3] > l_rect_yellow[2]*l_rect_yellow[3]:
        print("Found Blue Cube")
        return l_rect_blue
    else: 
        print("Found Green Cube")
        return l_rect_green