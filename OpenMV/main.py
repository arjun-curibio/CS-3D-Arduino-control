# Single Color Grayscale Blob Tracking Example
#
# This example shows off single color grayscale tracking using the OpenMV Cam
# and tracks the post and magnet in order to compute the stretch and beat frequency.
#
# Version: 2022_07_15

import sensor, image, time, math, mjpeg
import CuriBio_MV as cb
import utime
from pyb import Pin, LED, UART, USB_VCP
from ulab import numpy as np

pin7 = Pin('P7', Pin.IN, Pin.PULL_UP)  # Connected to pin 7
#green_led = LED(2)
red_led   = LED(1)


# Color Tracking Thresholds (Grayscale Min, Grayscale Max)
# The below grayscale threshold is set to only find extremely bright white areas.
thresholds = (245, 255)



uart = cb.Serial(3, 115200, timeout=0)
uart = cb.Serial(-1, 115200, timeout=25) # Set first argument to -1 to enable simulation mode
# usb = USB_VCP.init()

# if usb.isconnected():
#     uart.USB = True


val = 0
# Only blobs with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.

def magnet_post_cb(blob) -> bool:
    ### Return true if blob looks like a magnet or post
    ###

    # check magnet area thresholds + solidity
    # check post araa thresholds + circularity
    return True

mag_thresh = (0,40)
post_thresh = (0,10)

def get_framerate(tic, t0 = 0):
    toc = utime.ticks_ms() - t0
    if (toc > tic):
        frame_rate = 1000.0/(toc-tic)
    
    return frame_rate
def show_stretch_cytostretcher_MV(centroid_magnet=(254, 376),
                               centroid_post=(259, 213),
                               thresh_range=(mag_thresh, post_thresh),
                               area_range=((1000, 3000), (3000, 10000)),
                               outfile=None):
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.VGA)
    sensor.skip_frames(time = 1000)
    sensor.set_auto_gain(False) # must be turned off for color tracking
    
    sensor.set_auto_whitebal(False) # must be turned off for color tracking
    clock = time.clock()

    fps = 15  # Target FPS
    nframes = 30*fps  # Process 30 seconds worth

    ROI_post = (0, 250, 160, 320) # x1, y1, x2, y2
    ROI_magnet = (250, sensor.width(), 160, 320) # x1, y1, x2, y2
    
    CircularBufferSize = 3
    tracker = cb.PostAndMagnetTracker(thresh_range, area_range, roi_post = ROI_post, roi_magnet = ROI_magnet, CircularBufferSize=CircularBufferSize)
    
    print("\nmilliseconds\tstretch_percent\tbeat_freq\tlast_max_stretch\tfps")  # tab-delimited header
    frame_rate = 0
    t0 = utime.ticks_ms()
    running = True

    stretch = 0
    freq = 0
    max_stretch = 0
    value = ([stretch], [freq], [max_stretch])

    mag_thresh, post_thresh  = thresh_range
    wellLabels          = ['D',         'C',            'B',            'A']
    centroid_m          = [(0,0),       (0,0),          (0,0),          (0,0)]
    centroid_p          = [(0,0),       (0,0),          (0,0),          (0,0)]
    centroid_m_passive  = [(0,0),       (0,0),          (0,0),          (0,0)]
    centroid_p_passive  = [(0,0),       (0,0),          (0,0),          (0,0)]
    passive_length      = [-1,          -1,             -1,             -1]

    postManual          = [(53,237),    (53,237),       (53,237),       (53,237)]
    post_thresh         = [post_thresh, post_thresh,    post_thresh,    post_thresh]
    magnet_thresh       = [mag_thresh,  mag_thresh,     mag_thresh,     mag_thresh]
    postManualFlag      = [False,       False,          False,          False]
    initFlag            = [False,       False,          False,          False] # triggers high if need to init tracker
    isInit              = [False,       False,          False,          False]
    
    
    current_well = 0


    updatePostCentroid = False
    
    n_hysteresis = 5
    
    HELPERFLAG = False
    HELPERMASK = "POST"
    helper_magnet_thresh = (20,50)
    helper_post_thresh = (0,10)
    helper_magnet_area = (1500,3000)
    helper_post_area = (0,25000)
    helper_magnet_extent = 0.2
    helper_magnet_aspectratio = (0.9,10)
    helper_post_circularity = 0.4
    # for i in range(0, nframes):
    
    
    
    extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.GRAYSCALE)
    extra_fb.replace(sensor.snapshot())
    
    outstring = "-41#{}".format(t0)
    uart.write(outstring+'\n')
    
    # for i in range(0, nframes):
    while True:
        clock.tick()
        
        tic = utime.ticks_ms() - t0        
        img = sensor.snapshot()
        img.draw_string(50,100,"test")
        # uart inputs: (1) INIT, (2) CHANGE, (3) POST - does not work, (4) THRESH - does not work
        # val = usb.readline()
        # print(val)
        if uart.simulationMode == False:
            temp, command, info = uart.read()
            if command=='INIT':
                print(current_well)
                print(wellLabels[current_well])
                initFlag[current_well] = True
                current_well = temp
            elif command=='CHANGE':
                # set magnet threshold variable to what the tracker is currently
                magnet_thresh[current_well] = tracker.thresh_range[0]
                post_thresh[current_well] = tracker.thresh_range[1]
                current_well = temp # change well
                tracker.dist_neutral = passive_length[current_well] # update passive length
                tracker.thresh_range = (magnet_thresh[current_well], post_thresh[current_well]) # update tracker threshold to new well
                pass # do nothing, well change happens during read function
            elif command=='POSTMANUALTOGGLE':
                postManualFlag = True
            elif command=='POSTMANUAL':
                postManualFlag[current_well] = True
                postManual[current_well] = (int(info[0]), int(info[1]))
                print("POSTMANUAL, {}".format(postManual[current_well]))
            elif command=='RESET':
                initFlag = [False, False, False, False] # triggers high if need to init tracker
                isInit = [False, False, False, False]
                passive_length = [-1, -1, -1, -1]
            elif command=='MOTORINIT':
                if isInit[temp]:
                    outstring = '-51#{}#'.format(current_well)
                    uart.write(outstring + '\n')
                    print(outstring)
                
        else:
            if isInit[current_well] == False: # if not currently initialized
                initFlag[current_well] == True # trigger initialization
            else:
                initFlag[current_well] == False # hold low
        
        # First, query global stop/start input.  If high, do not do anything
        on = pin7.value()

        if uart.simulationMode == True:
            on = False # do not trigger global stop/start input
            if isInit[current_well] == False: # initialize, if uninitialized
                initFlag[current_well] = True
         
        if on: # if global start/stop input is on, do not continue to processing
            frame_rate = get_framerate(tic, t0)
            print('Stopped. fps = {}, exp = {}'.format(frame_rate, sensor.get_exposure_us()))
            continue
            
        if HELPERFLAG:
            h_thresh = (helper_magnet_thresh, helper_post_thresh)
            h_area = (helper_magnet_area, helper_post_area)
            h_extent = helper_magnet_extent
            h_ar = helper_magnet_aspectratio
            h_circ = helper_post_circularity
            
            stats_m, stats_p = cb.HelperFunction(img, h_thresh, h_area, h_ar, (ROI_magnet, ROI_post), h_extent, h_circ, HELPERMASK)
            continue

        if postManualFlag[current_well]:
            tracker.post_centroid = np.array(postManual[current_well])
            tracker.postOverride = True
            pass

        if initFlag[current_well]:
            initFlag[current_well] = False # immediately flip low
            tracker.passive_deflection = None # reset for initPassive function
            tracker.dist_neutral = None # reset for initPassive function
            tracker.maxTracker = cb.MaxTracker(200, CircularBufferSize, n_hysteresis)
            stats_m = cb.locate_magnet(img, tracker.thresh_range, tracker.area_range, ROI_magnet, extent = 0.7, aspectratio = (0.7, 2.0))
            stats_p = cb.locate_post(img, tracker.thresh_range, tracker.area_range, ROI_post, circularity=0.5)

            #print("GOT STATS")
            if len(stats_m) == 0 or len(stats_p) == 0:
                #print("GOING THROUGH DETERMINE THRESHOLDS")
                # run determine thresholds function to get new thresholds (needs more work to get more appropriate thresholds)
                
                thresh_range, tracker.area_range = cb.determineThresholds(img, tracker.area_range, tracker.roi, tracker.roi_post, tracker.roi_magnet, visualize=False)
                if uart.simulationMode and None in thresh_range:
                    pass
                else:
                    tracker.thresh_range = thresh_range
                # get new stats
                stats_m = cb.locate_magnet(img, tracker.thresh_range, tracker.area_range, ROI_magnet, extent = 0.7, aspectratio = (0.7, 2.0))
                stats_p = cb.locate_post(img, tracker.thresh_range, tracker.area_range, ROI_post, circularity=0.5)
                #print("PAST DETERMINE THRESHOLDS")
                # print(tracker.thresh_range)
            
            # put stats through initPassive function to set passive length variables in tracker and in script
            #print("GOING THROUGH INIT PASSIVE")
            centroid_m_passive[current_well], centroid_p_passive[current_well] = tracker.initPassive(stats_m, stats_p)

            # convert from numpy array to tuple for future use
            centroid_p_passive[current_well] = tuple(list(centroid_p_passive[current_well]))
            centroid_m_passive[current_well] = tuple(list(centroid_m_passive[current_well]))

            # extract passive length, thresholds, store for current well
            passive_length[current_well] = tracker.dist_neutral # dist_neutral = passive_length
            magnet_thresh[current_well] = tracker.thresh_range[0]
            post_thresh[current_well] = tracker.thresh_range[1]
            
            utime.sleep_ms(100)
            img = sensor.snapshot()
            # Modified processImage function, without plotting.  plotting_paramters might be duplicate of information already stored in tracker (or information that can be stored in tracker)
            value = tracker.processImage(img, tic, value, cb.PostAndMagnetTracker.computeStretchFrequency, postManualFlag[current_well], postManual[current_well])
            
            #print(value)
            if len(value[0]) != 0:
                stretch = value[0][-1]
            else:
                stretch = 0
            if stretch < -0.25 or stretch > 0.25:
                isInit[current_well] = False
                initFlag[current_well] = True # re-init
            else:
                isInit[current_well] = True # flip flag so that it runs through processImage now

            # output data to Arduino over Serial
            outstring = "-43#{}#{}#{}#{}#{}#{}".format(tic, current_well, passive_length[current_well], magnet_thresh[current_well], post_thresh[current_well], centroid_p_passive[current_well])
            print(outstring+",{},{}".format(frame_rate, tracker.maxTracker.update_max)) # print to IDE
            uart.write(outstring+'\n') # send to Arduino
            continue
            
        if isInit[current_well]:
            # Modified processImage function, without plotting.  plotting_paramters might be duplicate of information already stored in tracker (or information that can be stored in tracker)
            value = tracker.processImage(img, tic, value, cb.PostAndMagnetTracker.computeStretchFrequency, postManualFlag[current_well], postManual[current_well])
            
            if updatePostCentroid == True:
                centroid_p_passive[current_well] = tracker.centroid_p
                updatePostCentroid = False
            #print(tracker.centroid_m)
            if type(tracker.centroid_m) != np.array:
                tracker.centroid_m = np.array((250,250))
            ret = tracker.showTrackingOscilloscope(img, value, current_well)

            if ret != 0:
                continue

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
                
            outstring =  "-35#{}#{}#{}#{}".format(tic, round(stretch,1), tracker.maxTracker.foundMax, tracker.maxTracker.maxes.signal())
            #outstring =  "-35#{}#{}#{}#{}".format(tic, round(stretch,2), tracker.maxTrackerv2.foundMax, tracker.maxTracker.maxes.signal())
            
            print(outstring+",{},{},{},{},{},{},{}".format(round(frame_rate,1), tracker.maxTracker.hysteresis, tracker.maxTracker.avg_filter, tracker.maxTracker.update_max, tracker.maxTracker.last_min, tracker.maxTracker.last_max, tracker.maxTracker.v)) # print to IDE
            uart.write(outstring+'\n')
            
            
            frame_rate = get_framerate(tic, t0)
            tracker.setFps(frame_rate)
                
            tracker.maxTracker.foundMax = 0
            continue
            
        # If nothing happened (i.e. not initialized, not globally stopped, not in initializtion routine), just print fps
        frame_rate = get_framerate(tic, t0)

        tracker.setFps(frame_rate)  # Used to compute beat_frequency
        print("Nothing happened, {}, {}".format(frame_rate, current_well))
        outstring = '-47#{}'.format(tic)
        uart.write(outstring+'\n')


show_stretch_cytostretcher_MV(outfile=None)  # Use defaults
