import numpy as np
from scipy import linalg
import cv
import rospy

def classify(img, h_min, s_min, h_max, s_max):
    ''' Takes BRG opencv image. Returns uint8 array with class for each pixel. '''
    if type(img) == np.ndarray:
        img_hsv = np.zeros(img.shape, dtype=np.uint8)
    else:
        img_hsv = cv.CreateImage(cv.GetSize(img), 8, 3)

    cv.CvtColor(img, img_hsv, cv.CV_RGB2HSV)
    hsv_arr = np.array(img_hsv[:])
    mask = np.array((hsv_arr[...,0] >= h_min) & (hsv_arr[...,0] <= h_max) & \
      (hsv_arr[...,1] >= s_min) & (hsv_arr[...,1] <= s_max), dtype=np.uint8)
    return mask


