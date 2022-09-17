# Single Color Grayscale Blob Tracking Example
#
# This example shows off single color grayscale tracking using the OpenMV Cam
# and tracks the post and magnet in order to compute the stretch and beat frequency.
#
# Version: 2022_07_15


import sensor, image, time, math, mjpeg
import CuriBio_MV as cb
import utime
from pyb import Pin, LED, UART

pin7 = Pin('P7', Pin.IN, Pin.PULL_UP)  # Connected to pin 7
#green_led = LED(2)
red_led   = LED(1)


# Color Tracking Thresholds (Grayscale Min, Grayscale Max)
# The below grayscale threshold is set to only find extremely bright white areas.
thresholds = (245, 255)



uart = cb.Serial(3, 9600, timeout=25)

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
    ROI_magnet = (250, 580, 160, 320) # x1, y1, x2, y2
    
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
    current_well = 1
    wellLabels = ['A','B','C','D']
    passive_length = [-1, -1, -1, -1]
    plotting_parameters = (centroid_m[current_well], centroid_p[current_well], milliseconds, time_of_max, value)
    passiveLengthCalcFlag = False
    passive_deflection = 0


    initFlag = [False, False, False, False] # triggers high if need to init tracker
    isInit = [False, False, False, False]
    updatePostCentroid = False
    updateMagnetThreshold = False
    
    
    
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
    
    postManualFlag = False
    postManual = (100,250)
    
    extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.GRAYSCALE)
    extra_fb.replace(sensor.snapshot())
    
    while True:
        clock.tick()
        tic = utime.ticks_ms() - t0
        if uart.simulationMode == False:
            current_well, command, info = uart.read()
            if command=='INIT':
                initFlag[current_well] = True
            elif command=='CHANGE':
                pass # do nothing, well change happens during read function
            elif command=='POSTMANUALTOGGLE':
                postManualFlag = int(info[0])
            elif command=='POSTMANUAL':
                postManual = (int(info[0], int(info[1])))
        else:
            if isInit[current_well] == False: # if not currently initialized
                initFlag[current_well] == True # trigger initialization
            else:
                initFlag[current_well] == False # hold low

        # current_well = int(infos[0])
        # if uart.any():
        #     val = uart.read()
        #     #print(len(val))
        #     #print(val)
        #     #print(val.decode('utf-8'))
        #     if len(val) > 4: # in case of bad input
        #         val = val.decode('utf-8') # decode bytes into string
        #         #if val[-3] != '#':
        #             #continue
        #         val = val.split('#')[0] # split by occasional # at end
        #         infos = val.split('&') # split based on incoming delimiter
        #         print(infos)

        #         current_well = int(infos[0]) # always starts with integer - well number
        #         if infos[1] == 'INIT':
        #             initFlag[current_well] = True # flip initFlag high
        #         elif infos[1] == 'POSTMANUALTOGGLE':
        #             postManualFlag = bool(int(infos[2]))
        #         elif infos[1] == 'POSTMANUAL':
        #             postManual = (int(infos[2]), int(infos[3]))
        #         elif infos[1] == 'CHANGE':
        #             pass # only thing this updates is the well number
        #         elif infos[1] == 'POST':
        #             updatePostCentroid = True # flip flag for updating post centroid, (only graphical, does not change passive length)
        #         elif infos[1] == 'THRESH':
        #             new_magnet_threshold = (int(infos[2]),int(infos[3])) # extract new magnet threshold
        #             updateMagnetThreshold = True # flip magnet threshold updater
        #         elif infos[1] == 'HELPERTOGGLE':
        #             HELPERFLAG = int(infos[2]);
        #             helper_magnet_thresh = tracker.thresh_range[0]
        #             helper_post_thresh = tracker.thresh_range[1]
        #             helper_magnet_area = tracker.area_range[0]
        #             helper_post_area = tracker.area_range[1]
        #             helper_magnet_extent = 0.5
        #             helper_magnet_aspectratio = (0.9, 10)
        #             helper_post_circularity = 0.4
                    
        #         elif infos[1] == 'HELPER':
        #             # get values here
        #             # 2-3: magnet threshold
        #             # 4-5: post threshold
        #             # 6-7: magnet area
        #             # 8-9: post area
        #             # 10: magnet extent
        #             # 11: magnet aspect ratio
        #             # 12: post circularity
        #             helper_magnet_thresh = (int(infos[2]), int(infos[3]))
        #             helper_post_thresh = (int(infos[4]), int(infos[5]))
        #             helper_magnet_area = (int(infos[6]), int(infos[7]))
        #             helper_post_area = (int(infos[8]), int(infos[9]))
        #             helper_magnet_extent = float(infos[10])/100
        #             helper_magnet_aspectratio = (float(infos[11])/100, float(infos[12])/100)
        #             helper_post_circularity = float(infos[13])/100
        #             pass
        #         elif infos[1] == 'HELPERMASK':
        #             HELPERMASK = infos[2]
        #         # placeholders fort future input options
        #         elif infos[1] == 'POSTTHRESH':
        #             pass
        #         elif infos[1] == 'POSTAREA':
        #             pass
        #         elif infos[1] == 'MAGNETAREA':
        #             pass
        #         elif infos[1] == 'POSTAREA':
        #             pass
                
        
        # First, query global stop/start input.  If high, do not do anything
        on = pin7.value()
        if on:
            img = sensor.snapshot()
            toc = utime.ticks_ms() - t0
            if (toc > tic):
                frame_rate = 1000.0/(toc-tic)
            print('stopped. fps = {}'.format(frame_rate))
            continue
            
        if HELPERFLAG:
            img = sensor.snapshot()
            stats_m = cb.locate_magnet(img, (helper_magnet_thresh, helper_post_thresh), (helper_magnet_area, helper_post_area), ROI_magnet, helper_magnet_aspectratio, helper_magnet_extent)
            stats_p = cb.locate_post(img, (helper_magnet_thresh, helper_post_thresh), (helper_magnet_area, helper_post_area), ROI_post, helper_post_circularity)
            
            
            #img_copy = img.copy(copy_to_fb=True)
            ##img_left_mask = img_copy.mask_rectangle([ROI_post[0],ROI_post[1],ROI_post[2]-ROI_post[0],ROI_post[3]-ROI_post[1]])
            #mask_left = extra_fb.mask_rectangle(
            #img_left_mask = img.mask_rectangle([0, 160, 250, 160])
            #bw = img.binary([helper_magnet_thresh], False, True, mask=img_right_mask, copy=False)
            #bw.binary([helper_post_thresh], False, False, img_left_mask, copy=False)
            #bw.to_rainbow()
            
            if HELPERMASK=='POST':
                img.binary([helper_post_thresh], False, False)
            elif HELPERMASK=='MAGNET':
                img.binary([helper_magnet_thresh], False, False)
            else:
                pass
            img.to_rgb565()
            img.draw_rectangle(0, 0, sensor.width(), 150, (0,0,0), 0, True)
            img.draw_string(1,1,"MASKING: {}".format(HELPERMASK), scale=5, color=(0,255,0),x_spacing=-10)
            txt= """                     Magnet | Post
		   Threshold: (%5.0f,%5.0f) | (%5.0f,%5.0f)
		        Area: (%5.0f,%5.0f) | (%5.0f,%6.0f)
		      Extent:         %5.2f | 
		Aspect Ratio: (%5.2f,%5.2f) | (%5.2f,%5.2f)
		 Circularity:               | %4.2f
		""" % (helper_magnet_thresh[0], helper_magnet_thresh[1],helper_post_thresh[0],helper_post_thresh[1],
            helper_magnet_area[0], helper_magnet_area[1],helper_post_area[0],helper_post_area[1],
            helper_magnet_extent,
            helper_magnet_aspectratio[0], helper_magnet_aspectratio[1], 0.9, 1.1,
            helper_post_circularity)
            img.draw_string(1,50,txt, x_spacing=-1,y_spacing=-1, scale=1.5, color=(0,255,0))
            
            k=0
            #print("HELPER: {},{},{},{},{},{},{};".format(helper_magnet_thresh, helper_post_thresh, helper_magnet_area, helper_post_area, helper_magnet_extent, helper_magnet_aspectratio, helper_post_circularity), end=',')
            for blob in stats_m:
                if k==0:
                    th = 3
                    s = 5
                    if HELPERMASK == "MAGNET":
                        img.flood_fill(blob.cx(), blob.cy(), seed_threshold=0.05, color=(255,0,0))
                    else:
                        img.draw_rectangle(blob.rect(), thickness=th, color=(255,0,0))
                    #print("{},{},{},{},{},{},{}".format(blob[0],blob[1],blob[2],blob[3], blob.pixels(), blob.extent(), cb.getAspectRatio(blob)), end=',')
                else:
                    th = 1
                    s = 2
                img.draw_rectangle(blob.rect(), thickness=th, color=(255,0,0))
                
                #img.flood_fill(blob.cx(), blob.cy(), color=(255,0,0))
                img.draw_circle(blob.cx(), blob.cy(), s, color=(255,155,0), fill=True)  # marker
                #if blob.elongation() < 0.5:
                    #img.draw_edges(blob.min_corners(), thickness=th, color=(255,0,0))
                    #img.draw_line(blob.major_axis_line(), thickness=th, color=(255,0,0))
                    #img.draw_line(blob.minor_axis_line(), thickness=th, color=(255,0,0))
                # These values are stable all the time.
                #img.draw_rectangle(blob.rect(), thickness=th, color=(255,0,0))
                #img.draw_cross(blob.cx(), blob.cy(), thickness=th, color=(255,0,0))
                #img.draw_string(blob.cx()+1, blob.cy()+1, "AR: \n"+str(round(cb.getAspectRatio(blob),2)), scale=2, color=(255,155,0))
                #img.draw_string(blob.cx()+1, blob.cy()+1, "E: \n"+str(round(blob.extent(),2)), scale=2, color=(255,155,0))
                #img.draw_string(blob.cx()+1, blob.cy()+1, "A: \n"+str(round(blob.pixels(),2)), scale=2, color=(255,155,0))
                #img.draw_string(blob.cx()+1, blob.cy()+1, "Elong: \n"+str(round(blob.elongation(),2)), scale=2, color=(255,155,0))
                #img.draw_string(blob.cx()+1, blob.cy()+1, "Compact: \n"+str(round(blob.compactness(),2)), scale=2, color=(255,155,0))
                #img.draw_string(blob.cx()+1, blob.cy()+1, "Convex: \n"+str(round(blob.convexity(),2)), scale=2, color=(255,155,0))
                #img.draw_string(blob.cx()+1, blob.cy()+1, "Solid: \n"+str(round(blob.solidity(),2)), scale=2, color=(255,155,0))
                #img.draw_string(blob.cx()+1, blob.cy()+1, "Rot: \n"+str(round(blob.rotation_deg(),2)), scale=2, color=(255,155,0))
                img.draw_string(blob.cx()+1, blob.cy()+1, "E:%3.2f\nX:%3.2f\nAR:%3.2f" % (blob.elongation(), blob.extent(),cb.getAspectRatio(blob)), scale=1, color=(255,155,0))
                k += 1
                # Note - the blob rotation is unique to 0-180 only.
                # bw.draw_keypoints([(blob.cx(), blob.cy(), int(blob.rotation_deg()))], size=40, color=127)
            k=0
            for blob in stats_p:
                if k==0:
                    th = 3
                    s = 5
                    if HELPERMASK == "POST":
                        img.flood_fill(blob.cx(), blob.cy(), seed_threshold=0.03, color=(0,255,255))
                    else:
                        circ = blob.enclosing_circle()
                        img.draw_circle(circ[0], circ[1], circ[2] ,thickness=th, color=(0,255,255))
                    #print("{},{},{},{},{},{},{}".format(blob[0],blob[1],blob[2],blob[3], blob.pixels(), blob.roundness(), cb.getAspectRatio(blob)))
                else:
                    th = 1
                    s = 2
                img.draw_circle(blob.cx(), blob.cy(), s, color=(0,155,255), fill=True)  # marker
                #if blob.elongation() < 0.5:
                    #img.draw_edges(blob.min_corners(), thickness=th, color=(0,255,0))
                    #img.draw_line(blob.major_axis_line(), thickness=th, color=(0,255,0))
                    #img.draw_line(blob.minor_axis_line(), thickness=th, color=(0,255,0))
                ## These values are stable all the time.
                #img.draw_rectangle(blob.rect(), color=(255,255,0))
                #img.draw_cross(blob.cx(), blob.cy(), thickness=th, color=(0,255,0))
                #circ = blob.enclosing_circle()
                #img.draw_circle(circ[0], circ[1], circ[2] ,thickness=th, color=(0,255,0))
                #img.draw_string(blob.cx()+1, blob.cy()+1, "Area: \n"+str(blob.pixels()), scale=2, color=(0,155,255))
                img.draw_string(blob.cx()+1, blob.cy()+1, "AR: %4.2f\nCirc: %4.2f" % (cb.getAspectRatio(blob), blob.roundness()), scale=1, color=(0,155,255))
                k=k+1
                # Note - the blob rotation is unique to 0-180 only.
                # bw.draw_keypoints([(blob.cx(), blob.cy(), int(blob.rotation_deg()))], size=40, color=127)
            #print(' ')
            toc = utime.ticks_ms() - t0
            if (toc > tic):
                frame_rate = 1000.0/(toc-tic)
            #print('IN HELPER. values = {},{},{},{},{},{},{}. fps = {}'.format(helper_magnet_thresh, helper_post_thresh, helper_magnet_area, helper_post_area, helper_magnet_extent, helper_magnet_aspectratio, helper_post_circularity, frame_rate));
            #print('MASKING: {}'.format(HELPERMASK))
            continue
            
            
        if initFlag[current_well]:
            img = sensor.snapshot()
            initFlag[current_well] = False # immediately flip low
            tracker.passive_deflection = None # reset for initPassive function
            tracker.dist_neutral = None # reset for initPassive function

            stats_m = cb.locate_magnet(img, tracker.thresh_range, tracker.area_range, ROI_magnet, extent=0.7, aspectratio=(0.7, 2.0))
            stats_p = cb.locate_post(img, tracker.thresh_range, tracker.area_range, ROI_post, circularity=0.5)
            if len(stats_m) == 0 or len(stats_p) == 0:
                # run determine thresholds function to get new thresholds (needs more work to get more appropriate thresholds)
                tracker.thresh_range, tracker.area_range = cb.determineThresholds(img, tracker.area_range, tracker.roi, tracker.roi_post, tracker.roi_magnet, visualize=False)

                # get new stats
                stats_m = cb.locate_magnet(img, tracker.thresh_range, tracker.area_range, ROI_magnet, extent=0.7, aspectratio=(0.7, 2.0))
                stats_p = cb.locate_post(img, tracker.thresh_range, tracker.area_range, ROI_post, circularity=0.5)
                print(tracker.thresh_range)
            
            # put stats through initPassive function to set passive length variables in tracker and in script
            centroid_m_passive[current_well], centroid_p_passive[current_well] = tracker.initPassive(img, stats_m, stats_p, postManualFlag, postManual)

            # convert from numpy array to tuple for future use
            centroid_p_passive[current_well] = tuple(list(centroid_p_passive[current_well]))
            centroid_m_passive[current_well] = tuple(list(centroid_m_passive[current_well]))

            # extract passive length, thresholds, store for current well
            passive_length[current_well] = tracker.dist_neutral # dist_neutral = passive_length
            magnet_thresh[current_well] = tracker.thresh_range[0]
            post_thresh[current_well] = tracker.thresh_range[1]
            isInit[current_well - 1] = True # flip flag so that it runs through processImage now

            # output data to Arduino over Serial
            outstring = "-43&{}&{}&{}&{}&{}&{}".format(tic, current_well+1, passive_length[current_well], magnet_thresh[current_well], post_thresh[current_well], centroid_p_passive[current_well])

            #print(outstring+','+str(frame_rate)) # print to IDE
            uart.write(outstring+'\n') # send to Arduino

            continue

        if isInit[current_well]:
            img = sensor.snapshot()
            # if updateMagnetThreshold == True: # update magnet threshold with recieved range
            #     updateMagnetThreshold = False
            #     tracker.thresh_range = (new_magnet_threshold, tracker.thresh_range[1])

            # Modified processImage function, without plotting.  plotting_paramters might be duplicate of information already stored in tracker (or information that can be stored in tracker)
            value, plotting_parameters = tracker.processImage(img, tic, value, plotting_parameters, cb.PostAndMagnetTracker.computeStretch, postManualFlag, postManual)

            # values stored in plotting parameters:
            #  centroid_m
            #  centroid_p
            #  time (in milliseconds, not currently used)
            #  time_of_max (not currently used)
            #  value - duplicate of first output in processImage (not currently used)
            centroid_m[current_well], centroid_p[current_well], milliseconds, time_of_max, dummy = plotting_parameters

            if updatePostCentroid == True:
                centroid_p_passive[current_well] = centroid_p[current_well]
                updatePostCentroid = False

            # showTrackingOscilloscope takes in:
            #   img - object for IDE
            #   centroid_m
            #   centroid_p (in this case, centroid_p_passive)
            #   milliseconds
            #   time of max
            #   value - stretch, freq, last maximum
            #   well number
            ret, rotated = tracker.showTrackingOscilloscope(img, centroid_m[current_well], centroid_p[current_well], milliseconds, time_of_max, value, current_well)

            #img.draw_circle(centroid_m_passive[current_well].cx(), centroid_m_passive[current_well].cy(), 10, color=255, fill=True)
            #img.draw_circle(centroid_p_passive[current_well].cx(), centroid_p_passive[current_well].cy(), 15, color=155, fill=True)
            #img.draw_line(int(centroid_p[current_well][0]), int(centroid_p[current_well][1]), int(centroid_m[current_well][0]), int(centroid_m[current_well][1]), color=0, thickness=2)
            
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
                

            outstring =  "-35&{}&{}&{}".format(tic, round(stretch,3),current_well+1)

            print(outstring+', '+str(tracker.thresh_range)+str(frame_rate))
            uart.write(outstring+'\n')

            toc = utime.ticks_ms() - t0
            if toc > tic:
                frame_rate = 1000. / (toc-tic)
                tracker.setFps(frame_rate)
            continue

        img = sensor.snapshot()
        # If nothing happened (i.e. not initialized, not globally stopped, not in initializtion routine), just print fps
        toc = utime.ticks_ms() - t0
        if (toc > tic):
            frame_rate = 1000.0/(toc-tic)
            tracker.setFps(frame_rate)  # Used to compute beat_frequency
            print("Nothing happened, {}, {}, {}, {}".format(frame_rate, current_well+1, postManualFlag, postManual))


show_stretch_cytostretcher_MV(outfile=None)  # Use defaults
