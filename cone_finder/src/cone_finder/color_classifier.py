import time, shelve, sys, os, os.path
import numpy as np
from scipy import linalg
from sklearn import svm
import cv, cv_bridge
import rospy

class ColorClassifier:
    def __init__(self, training_dir):
        self.training_dir = training_dir

        # load training data
        self.X = []
        self.Y = []
        for shelf_name in os.listdir(self.training_dir):
            shelf_path = os.path.join(self.training_dir, shelf_name)
            try:
                s = shelve.open(shelf_path)
                self.X.extend(s['X'])
                self.Y.extend(s['Y'])
                rospy.logdebug('Loaded data from %s' % shelf_path)
            except:
                rospy.logdebug('Unable to load data from %s' % shelf_path)

        if len(self.X) == 0:
            rospy.logerr('ColorClassifier: Failed to load training data!')
            self.clf = None
            return

        # only use H and S
        self.X = np.array(self.X)[:,:2]
        self.Y = np.array(self.Y)

        # use an svm to classify pixels based on color
        self.clf = svm.SVC(kernel='rbf', gamma=0.005)
        self.clf.fit(self.X, self.Y)

    def classify(self, img):
        ''' Takes BRG opencv image. Returns uint8 array with class for each pixel. '''
        if self.clf == None:
            rospy.logerr('ColorClassifier: SVM not trained! Fail.')
            return None
        if type(img) == np.ndarray:
            img_hsv = np.zeros(img.shape, dtype=np.uint8)
        else:
            img_hsv = cv.CreateImage(cv.GetSize(img), 8, 3)            

        cv.CvtColor(img, img_hsv, cv.CV_RGB2HSV)
        hsv_arr = np.array(img_hsv[:])
        X_img = np.reshape(hsv_arr[:,:,:], (-1, 3))
        Y_img = self.clf.predict(X_img[:,:2]) # use only hue and saturation components
        class_arr = np.array(np.reshape(Y_img, hsv_arr.shape[:2]), dtype=np.uint8)
        return class_arr
