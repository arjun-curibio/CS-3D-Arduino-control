# Single Color Grayscale Blob Tracking Example
#
# This example shows off single color grayscale tracking using the OpenMV Cam
# and tracks the post and magnet in order to compute the stretch and beat frequency.

import sensor, image, time, math, mjpeg
import CuriBio_MV_winterrupts as cb
import utime
from pyb import Pin, LED, UART

pin7 = Pin('P7', Pin.IN, Pin.PULL_UP)  # Connected to pin 7
#green_led = LED(2)
red_led   = LED(1)
uart = UART(3, 9600, timeout = 1)
uart.init(9600, timeout = 1)

val = 0
# Color Tracking Thresholds (Grayscale Min, Grayscale Max)
# The below grayscale threshold is set to only find extremely bright white areas.
thresholds = (245, 255)

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
post_thresh_init = (0,17)
def show_stretch_cytostretcher_MV(centroid_magnet=(254, 376),
                               centroid_post=(259, 213),
                               thresh_range=(magnet_thresh_init, post_thresh_init),
                               area_range=((0, 2000), (3000, 10000)),
                               outfile=None):
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
    ROI_magnet	    = (    350,	sensor.width(),	160,	320)

    tracker = cb.PostAndMagnetTracker(fps, nframes, thresh_range, area_range, roi_post = ROI_post, roi_magnet = ROI_magnet)


    if outfile is not None:
        output = mpeg.Mjpeg(outfile)
    else:
        output = None

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
    current_well = 4
    wellLabels = ['A','B','C','D']
    passive_length = [-1, -1, -1, -1]
    plotting_parameters = (centroid_m, centroid_p, milliseconds, time_of_max, value)
    passiveLengthCalcFlag = False
    passive_deflection = 0

    initFlag = [False, False, False, True] # triggers high if need to init tracker
    isInit = [False, False, False, False]
    updatePostCentroid = False
    #for i in range(0, nframes):
    while (True):
        inputs = (passiveLengthCalcFlag, passive_length[current_well-1], passive_deflection)

        # uart inputs:
        #   <current well>&<command>&<information>
        #   <command> = 'INIT'
        #       initialze tracker, calculate passive length, return passive length
        #   <command> = 'CHANGE'
        #       change which well camera is under, update passive length, and thresholds
        #       <passive_len>&<post_thresh>&<mag_thresh>
        #   <command> = 'START'
        #       start sending feedback, toggle feedbackFlag=True
        #print(uart.any())
        if uart.any():
            val = uart.read()
            print(len(val))
            if len(val) > 4:
                val = val.decode('utf-8')
                print(val)
                val = val.split('#')[0]
                print(val)
                val = str(val)
                #print(val)
                infos = val.split('&')
                #print(infos)
                current_well = int(infos[0])
                if infos[1] == 'INIT':
                    initFlag[current_well - 1] = True
                elif infos[1] == 'CHANGE':
                    #print(infos)
                    passive_length[current_well-1]  = int(infos[2])
                    magnet_thresh[current_well-1]   = (int(infos[3]),int(infos[4]))
                    post_thresh[current_well-1]     = (int(infos[5]),int(infos[6]))
                    centroid_p_passive[current_well-1] = (int(infos[6]), int(infos[7]))
                elif infos[1] == 'POST':
                    updatePostCentroid = True

        #print(val)
        tic = utime.ticks_ms() - t0

        # clock.tick()
        img = sensor.snapshot()

        # Query the toggle button (pin7)
        activeFeedbackFlag = pin7.value()

        #print(initFlag)
        if initFlag[current_well-1]:
            initFlag[current_well-1] = False
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
            printstring += str(passive_length)
            printstring += '&'
            printstring += str(magnet_thresh)
            printstring += '&'
            printstring += str(post_thresh)
            printstring += '&'
            printstring += str(centroid_p_passive)

            print(printstring + str(frame_rate))

            uart.write(printstring + '\n')

            continue


        #print(on)
        if activeFeedbackFlag: # stop running tracker
            toc = utime.ticks_ms() - t0
            frame_rate = 1000.0/(toc-tic)
            print('stopped'+str(frame_rate))
            continue

        #if on and running and tic > 500:
            ##green_led.off()
            #red_led.on()
            #t0 = utime.ticks_ms()  # Debounce (reset)
            #print("----- Pause Processing -----")
            #running = False
            #continue

        ## Restart processing
        #elif on and not running and tic > 500:
            ##green_led.off()
            #red_led.off()
            #tracker = cb.PostAndMagnetTracker(fps, nframes, thresh_range, area_range, roi_post = ROI_post, roi_magnet = ROI_magnet)
            #print("\n---- Reset and go again ----")
            #print("\nmilliseconds\tstretch_percent\tbeat_freq\tlast_max_stretch\tfps")  # tab-delimited header
            #frame_rate = 0
            #t0 = utime.ticks_ms()
            #running = True
            #continue

        # Paused?

        if isInit[current_well-1]:
            value, plotting_parameters = tracker.processImage(img, tic, value, plotting_parameters, cb.PostAndMagnetTracker.computeStretch)
            #print(plotting_parameters)
            centroid_m[current_well-1], centroid_p[current_well-1], milliseconds, time_of_max, dummy = plotting_parameters
            #print(centroid_m[current_well-1], end=', ')
            #print(centroid_p[current_well-1])
            #print(centroid_m[current_well-1])
            if updatePostCentroid == True:
                centroid_p_passive[current_well-1] = centroid_p[current_well-1]
                updatePostCentroid = False

            ret, rotated = tracker.showTrackingOscilloscope(img, centroid_m[current_well-1], centroid_p_passive[current_well-1], milliseconds, time_of_max, value)
            #ret, rotated, value, plotting_parameters, inputs = tracker.processImage(img, tic, value, plotting_parameters, inputs, cb.PostAndMagnetTracker.computeStretchFrequency)
            #ret, rotated, value, plotting_parameters = tracker.processImage(img, tic, value, plotting_parameters, cb.PostAndMagnetTracker.computeStretchFrequency)

            passiveLengthCalcFlag, passive_len, passive_deflection = inputs
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

            printstring = "-35"
            printstring += '&'
            printstring += str(tic/1000)
            printstring += '&'
            printstring += str(stretch)
            printstring += '&'
            printstring += str(max_stretch)
            printstring += '&'
            printstring += str(current_well)
            printstring += '&'

            #printstring = str(inputs)

            print(printstring + str(frame_rate) + '#')
            uart.write(printstring + '#\n')

            toc = utime.ticks_ms() - t0
            if (toc > tic):
                frame_rate = 1000.0/(toc-tic)
                tracker.setFps(frame_rate)  # Used to compute beat_frequency
                #print(frame_rate)






show_stretch_cytostretcher_MV(outfile=None)  # Use defaults