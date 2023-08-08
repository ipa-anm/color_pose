import cv2 as cv2
import os
import numpy as np
import imutils

YELLOW_LOW = np.array([15, 20, 180])
YELLOW_HIGH = np.array([35, 255, 255])

BLUE_LOW = np.array([100, 70, 80])
BLUE_HIGH = np.array([130, 255, 255])

RED_LOW = np.array([135, 50, 80])
RED_HIGH = np.array([180, 255, 255])

GREEN_LOW = np.array([40, 30, 110])
GREEN_HIGH = np.array([80, 255, 255])

COLOR_RANGES = (YELLOW_LOW, YELLOW_HIGH, BLUE_LOW, BLUE_HIGH, RED_LOW, RED_HIGH, GREEN_LOW, GREEN_HIGH)
COLOR_NAMES = ("Yellow", "Blue", "Red", "Green")

    
# define the color range that will be detected in the image 
def define_color_range(low, high, img):
    # hsv = hue, saturation, value
    hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    low = low
    high = high
    mask = cv2.inRange(hsv_frame, low, high)
    color = cv2.bitwise_and(img, img, mask=mask)
    masked_img= mask.astype(np.uint8)
    return masked_img

# find countours of the masks
def find_contours(masked_img):
    cnts = cv2.findContours(
        masked_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    return cnts

# filter largest and second largest rectangle from the image with specified color
def filter_largest_rectangles(cnts):
    sum_width_height_second_largest = 0
    sum_width_height_largest = 0
    second_largest_rectangle = 0, 0, 0, 0
    largest_rectangle = 0, 0, 0, 0
    for contour in cnts:
        rect = cv2.boundingRect(contour)
        x, y, w, h = rect
        if w*h > sum_width_height_largest:
            sum_width_height_largest = w*h
            second_largest_rectangle = largest_rectangle
            largest_rectangle = rect
            continue
        if w*h > sum_width_height_second_largest: 
           sum_width_height_second_largest =w*h
           second_largest_rectangle = rect
    return second_largest_rectangle, largest_rectangle
    
    
# detect rectangular color ranges in the image  
def detect(img):
    color_array = [];
    for i in range (0, int(len(COLOR_RANGES)/2), 2):
        masked_img = define_color_range(COLOR_RANGES[i], COLOR_RANGES[i+1], img)
        contours = find_contours(masked_img)
        second_largest_rectangle, largest_rectangle = filter_largest_rectangles(contours)
        cv2.rectangle(img, (second_largest_rectangle[0], second_largest_rectangle[1]), (second_largest_rectangle[0]+second_largest_rectangle[2], second_largest_rectangle[1]+second_largest_rectangle[3]), (255,0,0), 2)
        cv2.rectangle(img, (largest_rectangle[0], largest_rectangle[1]), (largest_rectangle[0]+largest_rectangle[2], largest_rectangle[1]+largest_rectangle[3]), (255,0,0), 2)
        cv2.putText(img, COLOR_NAMES[int(i/2)], (largest_rectangle[0], largest_rectangle[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
        cv2.putText(img, COLOR_NAMES[int(i/2)], (second_largest_rectangle[0], second_largest_rectangle[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
        color_array.append(second_largest_rectangle)    
        color_array.append(largest_rectangle)    

        #cv2.imshow("Masks", img)
        #key = cv2.waitKey(1)

    return color_array, img