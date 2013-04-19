import shelve, os, os.path
from matplotlib import pyplot as plt
import numpy as np
import cv

import cone_finder.color_classifier

class TrainingGuiHSV:
    def __init__(self, training_dir, w=2):
        self.fig = plt.figure(figsize=(5, 4))
        self.ax = self.fig.add_subplot(121)
        self.ax.set_title('image')
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.w = w
        self.training_dir = training_dir
        self.X = np.zeros((0,3))
        self.Y = np.zeros((0,))
        self.data_ax = self.fig.add_subplot(122)

        # load previous training data
        self.X_prev = []
        self.Y_prev = []
        for shelf_name in os.listdir(self.training_dir):
            shelf_path = os.path.join(self.training_dir, shelf_name)
            try:
                s = shelve.open(shelf_path)
                self.X_prev.extend(s['X'])
                self.Y_prev.extend(s['Y'])                
                print 'Loaded training data from %s' % shelf_path
            except:
                print 'Unable to load training data from %s' % shelf_path
        self.X_prev = np.reshape(np.array(self.X_prev), (-1, 3))
        self.Y_prev = np.array(self.Y_prev)

        # choose a unique shelf name for the new data
        prev_shelf_nums = [int(shelf_name) for shelf_name in os.listdir(self.training_dir)]
        if len(prev_shelf_nums) == 0:
            self.shelf_name = '0'
        else:
            self.shelf_name = str(np.max(prev_shelf_nums) + 1)
        self.shelf_path = os.path.join(self.training_dir, self.shelf_name)
        print 'Chose shelf path %s' % self.shelf_path

        self.update_plots()

    def plot_image(self, img):
        self.img = img
        self.img_hsv = cv.CreateImage(cv.GetSize(self.img), 8, 3)
        cv.CvtColor(self.img, self.img_hsv, cv.CV_RGB2HSV)
        self.hsv_arr = np.array(self.img_hsv[:])
        self.ax.cla()
        self.ax.imshow(np.asarray(img[:])[...,[2,1,0]])
        self.fig.canvas.draw()

    def onclick(self, event):
        print 'button=%d, x=%d, y=%d, xdata=%f, ydata=%f' % (
            event.button, event.x, event.y, event.xdata, event.ydata)

        X_new = []
        Y_new = []
        for x in range(int(event.xdata) - self.w, int(event.xdata) + self.w):
            for y in range(int(event.ydata) - self.w, int(event.ydata) + self.w):
                X_new.append(self.hsv_arr[y,x,:])
                if event.button == 1:
                    Y_new.append(1.)
                elif event.button == 3:
                    Y_new.append(0.)
        self.X = np.vstack((self.X, X_new))
        self.Y = np.hstack((self.Y, Y_new))

        # save updated data to disk
        s = shelve.open(self.shelf_path)
        s['X'] = self.X
        s['Y'] = self.Y
        s.close()

        self.update_plots()
        
    def update_plots(self):
        ''' plot old and new data '''
        self.data_ax.cla()
        if len(self.X_prev) > 0:
            self.data_ax.scatter(self.X_prev[:,0], self.X_prev[:,1], color=plt.cm.jet(self.Y_prev))
        if len(self.X) > 0:
            self.data_ax.scatter(self.X[:,0], self.X[:,1], color=plt.cm.jet(self.Y))

        if len(self.X_prev) > 0:
            clf = cone_finder.color_classifier.ColorClassifier(self.training_dir)
            X_test = []
            for h in np.linspace(self.X_prev[:,0].min(), self.X_prev[:,0].max(), 40):
                for s in np.linspace(self.X_prev[:,1].min(), self.X_prev[:,1].max(), 40):
                    X_test.append((h, s))
            X_test = np.array(X_test)
            Y_test = clf.clf.predict(X_test)
            plt.scatter(X_test[:,0], X_test[:,1], color=plt.cm.jet(Y_test), marker='x', linewidth=3)
        
        plt.draw()
