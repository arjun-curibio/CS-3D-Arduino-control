# Single Color Grayscale Blob Tracking Example
#
# This example shows off single color grayscale tracking using the OpenMV Cam
# and tracks the post and magnet in order to compute the stretch and beat frequency.
#
# Version: 2022_07_15

import sensor, image, time, math, mjpeg, rpc, omv
import CuriBio_MV as cb
import utime
from pyb import Pin, LED, UART
from ulab import numpy as np

# omv.disable_fb()
pin7 = Pin('P7', Pin.IN, Pin.PULL_UP)  # Connected to pin 7
#green_led = LED(2)
red_led   = LED(1)


# Color Tracking Thresholds (Grayscale Min, Grayscale Max)
# The below grayscale threshold is set to only find extremely bright white areas.
thresholds = (245, 255)



uart = cb.Serial(3, 115200, timeout=0)
#uart = cb.Serial(-1, 115200, timeout=25) # Set first argument to -1 to enable simulation mode

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

magnet_thresh_init = (0,40)
post_thresh_init = (0,10)
def show_stretch_cytostretcher_MV(centroid_magnet=(254, 376),
                               centroid_post=(259, 213),
                               thresh_range=(magnet_thresh_init, post_thresh_init),
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
    ROI_magnet = (250, 600, 160, 320) # x1, y1, x2, y2
    
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
    centroid_m          = [(0,0),       (0,0),      (0,0),      (0,0)]
    centroid_p          = [(0,0),       (0,0),      (0,0),      (0,0)]
    centroid_m_passive  = [(0,0),       (0,0),      (0,0),      (0,0)]
    centroid_p_passive  = [(0,0),       (0,0),      (0,0),      (0,0)]
    post_thresh         = [post_thresh_init,post_thresh_init,post_thresh_init,post_thresh_init]
    magnet_thresh       = [magnet_thresh_init,magnet_thresh_init,magnet_thresh_init,magnet_thresh_init]
    
    
    milliseconds = 0
    time_of_max =  0
    current_well = 0
    wellLabels = ['D','C','B','A']
    passive_length = [-1, -1, -1, -1]
    plotting_parameters = (centroid_m, centroid_p, milliseconds, time_of_max, value)
    passiveLengthCalcFlag = False
    passive_deflection = 0
    
    
    area = tracker.area_range[0]
    extent = 0.7
    apectratio = 1.5
    magnet = None
    
    initFlag = [False, False, False, False] # triggers high if need to init tracker
    isInit = [False, False, False, False]
    updatePostCentroid = False
    
    n_hysteresis = 5
    ScreenMessage = [None, None, None, None]
    HELPERFLAG = False
    HELPERMASK = "POST"
    # for i in range(0, nframes):
    
    postManualFlag      = [     True,       True,       True,       True]
    postManual          = [ (53,237),   (53,237),   (53,237),   (53,237)]
    HomingMotorStatus   = [   "NONE",     "NONE",     "NONE",     "NONE"]

    extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.GRAYSCALE)
    extra_fb.replace(sensor.snapshot())
    
    outstring = "-41#{}".format(t0)
    uart.write(outstring+'\n')
    
    # for i in range(0, nframes):
    while True:
        clock.tick()
        tic = utime.ticks_ms() - t0        
        img = sensor.snapshot()
        # img.draw_string(50,100,"test")
        # uart inputs: (1) INIT, (2) CHANGE, (3) POST - does not work, (4) THRESH - does not work
        if uart.simulationMode == False:
            temp, command, info = uart.read()
            if command=='INIT':
                # print(current_well)
                # print(wellLabels[current_well])
                initFlag[current_well] = True
                current_well = temp
                
            elif command=='CHANGE':
                # set magnet threshold variable to what the tracker is currently
                magnet_thresh[current_well] = tracker.thresh_range[0]
                post_thresh[current_well] = tracker.thresh_range[1]
                centroid_m[current_well] = tracker.centroid_m
                current_well = temp # change well
                tracker.dist_neutral = passive_length[current_well] # update passive length
                tracker.thresh_range = (magnet_thresh[current_well], post_thresh[current_well]) # update tracker threshold to new well
                tracker.centroid_m = centroid_m[current_well]
                pass # do nothing, well change happens during read function
            elif command=='POSTMANUALTOGGLE':
                postManualFlag[current_well] = int(info[0])
            elif command=='POSTMANUAL':
                postManualFlag[current_well] = 1
                postManual[current_well] = (int(info[0]), int(info[1]))
            elif command=='RESET':
                initFlag = [False, False, False, False] # triggers high if need to init tracker
                isInit = [False, False, False, False]
                passive_length = [-1, -1, -1, -1]
            elif command=='MOTORINIT':
                HomingMotorStatus[current_well] = info[0]
                if HomingMotorStatus[current_well] == "HOME":
                    ScreenMessage[current_well] = "HOMING MOTOR."
                    img.draw_string(25,80, "HOMING MOTOR", scale=4, mono_space=False)
                elif HomingMotorStatus[current_well] == "FAIL":
                    ScreenMessage[current_well] = "FAILED MOTOR HOME."
                    img.draw_string(25, 80, "FAILED MOTOR HOME", scale=4, mono_space=False)
                elif HomingMotorStatus[current_well] == "PASS":
                    ScreenMessage[current_well] = "MOTOR READY."
                else:
                    pass
                
                
        else:
            if isInit[current_well] == False: # if not currently initialized
                initFlag[current_well] == True # trigger initialization
            else:
                initFlag[current_well] == False # hold low
        
        on = pin7.value()
        if uart.simulationMode == True:
            on = False # do not trigger global stop/start input
            if isInit[current_well] == False: # initialize, if uninitialized
                initFlag[current_well] = True
        
        # First, query global stop/start input.  If high, do not do anything
        
        if on:
            toc = utime.ticks_ms() - t0
            if (toc > tic):
                frame_rate = 1000.0/(toc-tic)
            print('Stopped. fps = {}'.format(frame_rate))
            continue
            
        if ScreenMessage[current_well] is not None:
            # img.draw_string(25, 80, ScreenMessage[current_well], scale=4, mono_space=False)
            pass
        
        if postManualFlag[current_well] == True:
            img.draw_circle(postManual[current_well][0], postManual[current_well][1], 10, color=127, fill=True)
        
        if initFlag[current_well]:
            img.draw_string(25, 100, "CAMERA INITIALIZING", scale=4)
            initFlag[current_well] = False # immediately flip low
            # RESET PASSIVE DEFLECTIONS, MAX TRACKER
            tracker.passive_deflection = None # reset for initPassive function
            tracker.dist_neutral = None # reset for initPassive function
            tracker.maxTracker = cb.MaxTracker(200, CircularBufferSize, n_hysteresis) # reset maxTracker

            # get magnet and post
            stats_m = cb.locate_magnet(img, tracker.thresh_range, tracker.area_range, ROI_magnet, extent = 0.7, aspectratio = (0.7, 2.0))
            if postManualFlag[current_well] == False:
                stats_p = cb.locate_post(img, tracker.thresh_range, tracker.area_range, ROI_post, circularity=0.5)
            else:
                stats_p = postManual[current_well]

            if len(stats_m) == 0:
                new_magnet_thresh, area_range = cb.determineThresholds(img, tracker.area_range[0], tracker.roi, 'magnet')
                tracker.thresh_range = (new_magnet_thresh, tracker.thresh_range[1])
                stats_m = cb.locate_magnet(img, tracker.thresh_range, tracker.area_range, ROI_magnet, extent = 0.7, aspectratio = (1,2))
            
            area = tracker.area_range[0]
            extent = 0.7
            apectratio = 1.5
            if len(stats_m) > 0:
                # choose best stats
                magnet = stats_m[0]
                area = magnet.area()
                cent_m = np.array((magnet.cx(), magnet.cy()))
                extent = magnet.extent()
                aspectratio = float(magnet.w()) / float(magnet.h())
                pass
            else:
                magnet = None
                area = tracker.area_range[0][1]
                cent_m = np.array((0,0))
                extent = tracker.extent
                aspectratio = tracker.aspectratio
            
            
            # put stats through initPassive function to set passive length variables in tracker and in script
            centroid_m_passive[current_well], centroid_p_passive[current_well] = tracker.initPassive(stats_m, stats_p, postManualFlag[current_well], postManual[current_well])

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
            
            stretch = value[0][-1]
                
            if stretch < -0.25 or stretch > 0.25:
                isInit[current_well] = False
                initFlag[current_well] = True # re-init
            else:
                isInit[current_well] = True # flip flag so that it runs through processImage now

            # output data to Arduino over Serial
            outstring = "-43#{}#{}#{}#{}#{}#{}".format(tic, current_well, passive_length[current_well], magnet_thresh[current_well], post_thresh[current_well], centroid_p_passive[current_well])
            #print(outstring+",{},{}".format(frame_rate, tracker.maxTracker.update_max)) # print to IDE
            uart.write(outstring+'\n') # send to Arduino
            continue
            
        if isInit[current_well]:
            # Modified processImage function, without plotting.  plotting_paramters might be duplicate of information already stored in tracker (or information that can be stored in tracker)
            value = tracker.processImage(img, tic, value, cb.PostAndMagnetTracker.computeStretchFrequency, postManualFlag[current_well], postManual[current_well])
            
            if updatePostCentroid == True:
                centroid_p_passive[current_well] = tracker.centroid_p[current_well]
                updatePostCentroid = False
            # showTrackingOscilloscope takes in:
            #   img - object for IDE
            #   centroid_m
            #   centroid_p (in this case, centroid_p_passive)
            #   milliseconds
            #   time of max
            #   value - stretch, freq, last maximum
            #   well number
            ret, rotated = tracker.showTrackingOscilloscope(img, value, current_well)

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
            
            #print(outstring+",{},{},{},{},{},{},{}".format(round(frame_rate,1), tracker.maxTracker.hysteresis, tracker.maxTracker.avg_filter, tracker.maxTracker.update_max, tracker.maxTracker.last_min, tracker.maxTracker.last_max, tracker.maxTracker.v)) # print to IDE
            uart.write(outstring+'\n')
            
            if len(tracker.stats_m) > 0:
                foundMagnet = True
                area = tracker.stats_m[0].area()
                cent_m = tracker.centroid_m
                extent = tracker.stats_m[0].extent()
                aspectratio = float(tracker.stats_m[0].w()) / float(tracker.stats_m[0].h())
                
                magnet = tracker.stats_m[0]
                statsBox = [magnet.cx()-int(tracker.sizeStats/2), 
                            magnet.cy()-int(tracker.sizeStats/2), 
                            tracker.sizeStats,
                            tracker.sizeStats]
                s = img.get_statistics(roi=statsBox)
                thresh = int(s.median()*tracker.smudgeFactor)
                
                tracker.area_range = ((int(float(tracker.stats_m[0].area())*0.8), int(float(tracker.stats_m[0].area())*1.2)), tracker.area_range[1])
                tracker.extent = extent*0.9
                tracker.aspectratio = (aspectratio*0.8, aspectratio*1.2)
                
                if thresh < tracker.thresh_range[0][1] - 1:
                    magnet_threshold = (0, tracker.thresh_range[0][1] - 1)
                elif thresh > tracker.thresh_range[0][1] + 1:
                    magnet_threshold = (0, tracker.thresh_range[0][1] + 1)
                else:
                    magnet_threshold = tracker.thresh_range[0]
                tracker.thresh_range = (magnet_threshold, tracker.thresh_range[1])
                
                mag_roi = ( magnet.cx()-int(tracker.sizeROIy/2), 
                            magnet.cx()+int(tracker.sizeROIy/2), 
                            magnet.cy()-int(tracker.sizeROIx/2), 
                            magnet.cy()+int(tracker.sizeROIx/2))
                bbox = magnet.rect()
                tracker.roi_magnet = mag_roi
                #self.thresh_range = (mag_thresh, self.thresh_range[1])
                img.draw_rectangle(mag_roi[0],mag_roi[2],mag_roi[1]-mag_roi[0],mag_roi[3]-mag_roi[2],color=255, thickness=2)
                
                #stats_m = locate_magnet(img, self.thresh_range, self.area_range, mag_roi, self.aspectratio, self.extent)
                img.draw_rectangle(bbox[0], bbox[1], bbox[2], bbox[3], color=155, thickness=2)
                print(""" {}  area = {} ({}) centroid_m = {},{} extent = {:3.2f} ({:3.2f}) aspectratio = {:3.2f} ({:2.1f}, {:2.1f}) thresh = {} ({})   {}""".format(tic, 
                            area, tracker.area_range[0][1], 
                            cent_m[0],cent_m[1], 
                            extent, tracker.extent,
                            aspectratio, tracker.aspectratio[0],tracker.aspectratio[1],
                            thresh, tracker.thresh_range[0][1],s))
                #print("{}".format(frame_rate))
                #tracker.stats_m = []
            
            else:
                foundMagnet = False
                print("{} area = {} centroid_m = {},{} extent = {} aspectratio = {} thresh = {}".format(tic, tracker.area_range[0][1], cent_m[0],cent_m[1], tracker.extent, tracker.aspectratio, tracker.thresh_range[0][1]))
                img.draw_string(88,419,'WARNING',[255,255,255], scale=2)
                
                
            #print(tracker.stats_m)
            #tracker.stats_m = []
            toc = utime.ticks_ms() - t0
            if toc > tic:
                frame_rate = 1000. / (toc-tic)
                tracker.setFps(frame_rate)
                
            tracker.maxTracker.foundMax = 0
            continue
            
        # If nothing happened (i.e. not initialized, not globally stopped, not in initializtion routine), just print fps
        toc = utime.ticks_ms() - t0
        if (toc > tic):
            frame_rate = 1000.0/(toc-tic)
            tracker.setFps(frame_rate)  # Used to compute beat_frequency
            #print("Nothing happened, {}, {}".format(frame_rate, current_well))
            outstring = '-47#{}'.format(tic)
            uart.write(outstring+'\n')


show_stretch_cytostretcher_MV(outfile=None)  # Use defaults
