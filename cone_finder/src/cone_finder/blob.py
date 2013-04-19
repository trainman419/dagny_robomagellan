import sys, random
import numpy as np
import cv

class Blob:
    '''
    Simple class that represents a contiguous blob of pixels in an image.
    '''
    def __init__(self, contour):
        self.area = cv.ContourArea(contour)
        self.contour_pixels = np.array(contour[:])
        self.bbox_x0, self.bbox_y0, self.bbox_width, self.bbox_height = cv.BoundingRect(contour)

    def get_area(self):
        return self.area

    def get_center(self):
        ''' Return center of bounding box which contains the blob '''
        return self.bbox_x0 + self.bbox_width/2.0, self.bbox_y0 + self.bbox_height/2.0

def threshold(min_vals, max_vals, img):
    '''
    Threshold image, returning a binary image of the pixels which are in the given range.

    min_vals - sequence with minimum values for each channel
    max_vals - sequence with maximum values for each channel
    img - image with channels in the same order as values in the min and max sequences

    returns: a uint8 array which has ones for each pixel within the
    range given, and zeros otherwise
    '''
    img_arr = np.asarray(img[:])
    bin_arr = np.ones((img.height, img.width), dtype=np.bool)
    for channel_i in range(img_arr.shape[2]):
        bin_arr *= img_arr[:,:,channel_i] >= min_vals[channel_i]
        bin_arr *= img_arr[:,:,channel_i] <= max_vals[channel_i]
    return np.array(bin_arr, dtype=np.uint8)

def get_blobs(bin_arr):
    '''
    Find all contiguous nonzero blobs in the image, and return a list of Blob objects.
    '''
    bin_img = cv.fromarray(bin_arr.copy())
    storage = cv.CreateMemStorage(0)
    contours = cv.FindContours(bin_img, storage, cv.CV_RETR_EXTERNAL, cv.CV_CHAIN_APPROX_NONE)
    blobs = []
    while contours:
        blobs.append(Blob(contours))
        contours = contours.h_next()
    return sorted(blobs, key=Blob.get_area, reverse=True)

def draw_blobs(blobs):
    '''
    Uses pyplot to plot a list of Blob objects, with colors corresponding to their area.
    '''
    from matplotlib import pyplot as plt
    for blob in blobs:
        #color = plt.cm.jet(blob.area/255.)
        color = 'g'
        x_list = list(blob.contour_pixels[:,0]) + [blob.contour_pixels[0,0]]
        y_list = list(blob.contour_pixels[:,1]) + [blob.contour_pixels[0,1]]
        plt.plot(x_list, y_list, '-', color=color)
