# Single Color Grayscale Blob Tracking Example
#
# This example shows off single color grayscale tracking using the OpenMV Cam
# and tracks the post and magnet in order to compute the stretch and beat frequency.

import sensor, image, time, math, mjpeg
import CuriBio_MV_winterrupts as cb
from ulab import numpy as np
import utime
from pyb import Pin, LED, UART

pin7 = Pin('P7', Pin.IN, Pin.PULL_UP)  # Connected to pin 7
green_led = LED(2)
red_led   = LED(1)
#LED(1).on()
#LED(2).on()
LED(3).on()

val = 0
# Color Tracking Thresholds (Grayscale Min, Grayscale Max)
# The below grayscale threshold is set to only find extremely bright white areas.
thresholds = (245, 255)

# Only blobs with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.


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
        self.passive_length = None

        self.isInit = False

class SerialInput:
    def __init__(self, hwID=3, baudrate=9600, timeout=25):
        self.uart = UART(hwID, baudrate, timeout=timeout)
        self.uart.init(baudrate, timeout=timeout)
        self.val = ''
    def getLine(self):
        self.val = ''
        self.infos = []

        if self.uart.any():
            val = self.uart.read()
            if len(val) > 4:
                val = val.decode('utf-8')
                val = val.split('#')[0]
                infos = val.split('&')
                current_well = infos.pop(0)
                command = infos.pop(0)
                self.val = val
                self.infos = infos
                return current_well, command
            else:
                return -1, -1
        else:
            return -1, -1
    def sendLine(self, magic, values):
        values = [str(values[i]) for i in range(len(values))]
        stringval = str(magic)+'&'+('&'.join(values))+'\n'
        self.uart.print(printstring)



magnet_thresh_init = (0,40)
post_thresh_init = (0,10)
def show_stretch_cytostretcher_MV(centroid_magnet=(254, 376),
                               centroid_post=(259, 213),
                               thresh_range=(magnet_thresh_init, post_thresh_init),
                               area_range=((1000, 3000), (3000, 10000))):
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.VGA)
    sensor.skip_frames(time = 1000)
    sensor.set_auto_gain(False) # must be turned off for color tracking
    sensor.set_auto_whitebal(False) # must be turned off for color tracking
    clock = time.clock()

    #global magnet_thresh
    #global post_thresh
    fps = 15  # Target FPS
    nframes = 30*fps  # Process 30 seconds worth
    print(sensor.height())
    ROI_post	    = (	0,	250,			160,	320) # x, y, w, h
    ROI_magnet	    = (    250,	sensor.width(),	160,	320)

    tracker = cb.PostAndMagnetTracker(fps, nframes, thresh_range, area_range, roi_post = ROI_post, roi_magnet = ROI_magnet)
    print("\nmilliseconds\tstretch_percent\tbeat_freq\tlast_max_stretch\tfps")  # tab-delimited header
    frame_rate = 0
    t0 = utime.ticks_ms()
    running = True
    stretch = 0
    freq = 0
    max_stretch = 0
    value = ([stretch], [freq], [max_stretch])
    centroid_m          = [(0,0),       (0,0),      (0,0),      (0,0)]
    centroid_p          = [(0,0),       (0,0),      (0,0),      (0,0)]
    centroid_m_passive  = [(0,0),       (0,0),      (0,0),      (0,0)]
    centroid_p_passive  = [(0,0),       (0,0),      (0,0),      (0,0)]
    post_thresh         = [post_thresh_init,post_thresh_init,post_thresh_init,post_thresh_init]
    magnet_thresh       = [magnet_thresh_init,magnet_thresh_init,magnet_thresh_init,magnet_thresh_init]

    milliseconds = 0
    time_of_max =  0
    current_well = 1
    wellLabels = ['A','B','C','D']
    passive_length = [-1, -1, -1, -1]
    plotting_parameters = (centroid_m, centroid_p, milliseconds, time_of_max, value)
    passiveLengthCalcFlag = False
    passive_deflection = 0

    initFlag = [False, False, False, False] # triggers high if need to init tracker
    isInit = [False, False, False, False]
    updatePostCentroid = False
    updateMagnetThreshold = False
    wells = [Tissue(i, magnet_thresh_init, post_thresh_init, None, None) for i in range(4)]
    ser = SerialInput(3, 9600, 25)
    #for i in range(0, nframes):
    while (True):
        inputs = (passiveLengthCalcFlag, passive_length[current_well-1], passive_deflection)


        command = ''
        current_well, command = ser.getLine()
        wellidx = current_well - 1
        if command == 'INIT':
            initFlag[wellidx] = True
        elif command == 'CHANGE':
            pass
        elif command == 'POST':
            updatePostCentroid = True
            pass
        elif command == 'THRESH':
            new_magnet_threshold = int(ser.infos[0], ser.infos[1])
            updateMagnetThreshold = True
            pass
        # placeholders fort future input options
        elif command == 'POSTTHRESH':
            pass
        elif command == 'POSTAREA':
            pass
        elif command == 'MAGNETAREA':
            pass
        elif command == 'POSTAREA':
            pass

        #print(val)
        tic = utime.ticks_ms() - t0

        # clock.tick()
        img = sensor.snapshot()

        # Query the toggle button (pin7)
        activeFeedbackFlag = pin7.value()

        #print(initFlag)
        if activeFeedbackFlag: # stop running tracker
            toc = utime.ticks_ms() - t0
            if (toc != tic):
                frame_rate = 1000.0/(toc-tic)
            print('stopped'+str(frame_rate))
            continue

        if initFlag[current_well-1]:
            initFlag[current_well-1] = False
            tracker.passive_deflection = None
            tracker.dist_neutral = None  # distance in pixels
            stats_m = cb.locate_magnet(img, tracker.thresh_range, tracker.area_range, ROI_magnet)
            stats_p = cb.locate_post(img, tracker.thresh_range, tracker.area_range, ROI_post)
            if len(stats_m) == 0 or len(stats_p) == 0:
                print(len(stats_m))
                print(len(stats_p))
                tracker.thresh_range, tracker.area_range = cb.determineThresholds(img, tracker.area_range, tracker.roi, tracker.roi_post, tracker.roi_magnet, visualize=False)
                stats_m = cb.locate_magnet(img, tracker.thresh_range, tracker.area_range, ROI_magnet)
                stats_p = cb.locate_post(img, tracker.thresh_range, tracker.area_range, ROI_post)
                print(tracker.thresh_range)
            centroid_m_passive[current_well-1], centroid_p_passive[current_well-1] = tracker.initPassive(img, stats_m, stats_p)
            centroid_p_passive[current_well-1] = tuple(list(centroid_p_passive[current_well-1]))
            centroid_m_passive[current_well-1] = tuple(list(centroid_m_passive[current_well-1]))

            passive_length[current_well-1] = tracker.dist_neutral
            print(tracker.dist_neutral)
            magnet_thresh[current_well-1] = tracker.thresh_range[0]
            post_thresh[current_well-1] = tracker.thresh_range[1]
            isInit[current_well - 1] = True

            printstring = "-43"
            printstring += '&'
            printstring += str(tic/1000)
            printstring += '&'
            printstring += str(current_well)
            printstring += '&'
            printstring += str(passive_length[current_well-1])
            printstring += '&'
            printstring += str(magnet_thresh[current_well-1])
            printstring += '&'
            printstring += str(post_thresh[current_well-1])
            printstring += '&'
            printstring += str(centroid_p_passive[current_well-1])

            print(printstring + str(frame_rate))

            ser.sendLine(-43, [tic, wellidx, passive_length[wellidx], magnet_threshold[wellidx], post_threshold[wellidx], centroid_p_passive[wellidx])
            #uart.write(printstring + '\n')

            continue

        if isInit[current_well-1]:
            tracker.thresh_range = (magnet_thresh[current_well-1], post_thresh[current_well-1])
            if updateMagnetThreshold == True:
                updateMagnetThreshold = False
                tracker.thresh_range = (new_magnet_threshold, tracker.thresh_range[1])

            value, plotting_parameters = tracker.processImage(img, tic, value, plotting_parameters, cb.PostAndMagnetTracker.computeStretch)
            #print(plotting_parameters)
            centroid_m[current_well-1], centroid_p[current_well-1], milliseconds, time_of_max, dummy = plotting_parameters
            #print(centroid_m[current_well-1], end=', ')
            #print(centroid_p[current_well-1])
            #print(centroid_m[current_well-1])
            if updatePostCentroid == True:
                centroid_p_passive[current_well-1] = centroid_p[current_well-1]
                updatePostCentroid = False

            #print(current_well)
            ret, rotated = tracker.showTrackingOscilloscope(img, centroid_m[current_well-1], centroid_p_passive[current_well-1], milliseconds, time_of_max, value, current_well)
            #ret, rotated, value, plotting_parameters, inputs = tracker.processImage(img, tic, value, plotting_parameters, inputs, cb.PostAndMagnetTracker.computeStretchFrequency)
            #ret, rotated, value, plotting_parameters = tracker.processImage(img, tic, value, plotting_parameters, cb.PostAndMagnetTracker.computeStretchFrequency)

            #passiveLengthCalcFlag, passive_len, passive_deflection = inputs
            #value = ((0),(0),(0))
            ret = 0
            if ret != 0:
                continue
            if output is not None:
                output.add_frame(img)


            if  len(value[0]) > 0 and len(value[1]) > 0:
                stretch = value[0][-1]
                freq = value[1][-1]
                max_stretch = value[2]
            elif len(value[0]) > 0 and len(value[1]) == 0:
                stretch = value[0][-1]
                freq = 'Error'
                max_stretch = 'Error'
            elif len(value[0]) == 0 and len(value[1]) >= 0:
                stretch = 'Error'
                freq = value[1][-1]
                max_stretch = 'Error'
            else:
                stretch = 'Error'
                freq = 'Error'
                max_stretch = 'Error'

            printstring = "-35&{}&{}&{}".format(round(tic/1000,3), round(stretch,3),current_well)
            #printstring = "-35"
            #printstring += '&'
            #printstring += "{}".format(round(tic/1000,1))
            #printstring += '&'
            #printstring += str(stretch)
            #printstring += '&'
            #printstring += str(current_well)
            #printstring += '&'

            #printstring = str(inputs+str(frame_rate))

            print(printstring + str(tracker.thresh_range)+'&'+str(initFlag[current_well-1])+'&'+str(isInit[current_well-1])+'&'+str(frame_rate) + '#')

            uart.write(printstring + '\n')

            toc = utime.ticks_ms() - t0
            if (toc > tic):
                frame_rate = 1000.0/(toc-tic)
                tracker.setFps(frame_rate)  # Used to compute beat_frequency
                #print(frame_rate)
            continue

        toc = utime.ticks_ms() - t0
        if (toc > tic):
            frame_rate = 1000.0/(toc-tic)
            tracker.setFps(frame_rate)  # Used to compute beat_frequency
            print("Nothing happened, {}, {}, {}, {}".format(frame_rate, initFlag, isInit, current_well))






show_stretch_cytostretcher_MV()  # Use defaults
