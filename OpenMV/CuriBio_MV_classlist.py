# CuriBio_MV module
# import cv2
# from scipy import ndimage
# from scipy.spatial import distance
# import matplotlib.pyplot as mp
import sensor, image, time, math, utime
from pyb import Pin, LED
from ulab import numpy as np

class Tissue:
    def __init__(self, wellID, mag_thresh, post_thresh, post_centroid, magnet_centroid=None):
        self.mag_thresh = mag_thresh
        self.post_thresh = post_thresh
        self.well_ID = wellID
        self.post_centroid = post_centroid
        self.magnet_centroid = magnet_centroid

        self.stats_m = []
        self.stats_p = []
        self.magnet_area = 0
        self.post_area = 0
        self.passive_length = np.linalg.norm(self.post_centroid - self.magnet_centroid)
        
        self.isInit = False
        
        pass