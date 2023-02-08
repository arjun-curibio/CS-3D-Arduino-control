import sensor, image, time, math, mjpeg, omv, utime
from CuriBio_MV import *
from pyb import Pin, LED, UART
from ulab import numpy as np


sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.VGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time = 1000)


extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.GRAYSCALE)
extra_fb.replace(sensor.snapshot())

pin7 = Pin('P7', Pin.IN, Pin.PULL_UP)

uart = Serial(3, 115200, timeout=0)
clock = time.clock()

uart.simulationMode = False

# INITIAL VALUES
ROI_post = (0, 250, 160, 320)
ROI_magnet = (250, 600, 160, 320)

frame_rate = 0
t0 = utime.ticks_ms()

stretch, freq, max_stretch = [0],[0],[0]
value = (stretch, freq, max_stretch)

# ROW PARAMTERS (n=4)
row_labels      = ['D',     'C',    'B',    'A']

blob_centroids = [  {"magnet_centroid":(0,0), "post_centroid": (53,237), "passive_magnet_centroid": (0,0), "passive_post_centroid": (53,237)},
                    {"magnet_centroid":(0,0), "post_centroid": (53,237), "passive_magnet_centroid": (0,0), "passive_post_centroid": (53,237)},
                    {"magnet_centroid":(0,0), "post_centroid": (53,237), "passive_magnet_centroid": (0,0), "passive_post_centroid": (53,237)},
                    {"magnet_centroid":(0,0), "post_centroid": (53,237), "passive_magnet_centroid": (0,0), "passive_post_centroid": (53,237)}]

magnet_tracking_parameters = [  {'area':(1000,3000),'extent':0.6,'aspectratio':(1,3),'threshold':(0,40), 'roi':ROI_magnet},
                                {'area':(1000,3000),'extent':0.6,'aspectratio':(1,3),'threshold':(0,40), 'roi':ROI_magnet},
                                {'area':(1000,3000),'extent':0.6,'aspectratio':(1,3),'threshold':(0,40), 'roi':ROI_magnet},
                                {'area':(1000,3000),'extent':0.6,'aspectratio':(1,3),'threshold':(0,40), 'roi':ROI_magnet}]

post_tracking_parameters = [    {'area':(3000,15000),'circularity':0.6,'threshold':(0,10), 'roi':ROI_post, 'manual_flag':True, 'post_manual':(53,237)},
                                {'area':(3000,15000),'circularity':0.6,'threshold':(0,10), 'roi':ROI_post, 'manual_flag':True, 'post_manual':(53,237)},
                                {'area':(3000,15000),'circularity':0.6,'threshold':(0,10), 'roi':ROI_post, 'manual_flag':True, 'post_manual':(53,237)},
                                {'area':(3000,15000),'circularity':0.6,'threshold':(0,10), 'roi':ROI_post, 'manual_flag':True, 'post_manual':(53,237)}]

tissue_parameters = [   {"passive_length":100},
                        {"passive_length":100},
                        {"passive_length":100},
                        {"passive_length":100}]

motor_statuses = [{"homing_status": 0, "enabled: ":False, "position": 0},
                  {"homing_status": 0, "enabled: ":False, "position": 0},
                  {"homing_status": 0, "enabled: ":False, "position": 0},
                  {"homing_status": 0, "enabled: ":False, "position": 0}]

camera_initialized = [  {"begin_initialization": False, "is_initialized": False},
                        {"begin_initialization": False, "is_initialized": False},
                        {"begin_initialization": False, "is_initialized": False},
                        {"begin_initialization": False, "is_initialized": False}]


# blob_centroids = {"magnet_centroid":(0,0), "post_centroid": (53,237), "passive_magnet_centroid": (0,0), "passive_post_centroid": (53,237)}
# magnet_tracking_parameters = {'area':(1000,3000),'extent':0.6,'aspectratio':(1,3),'threshold':(0,40), 'roi':ROI_magnet}
# post_tracking_parameters = {'area':(3000,15000),'circularity':0.6,'threshold':(0,10), 'roi':ROI_post, 'manual_flag':True, 'post_manual':(53,237)}
# tissue_parameters = {"passive_length":100}
# motor_statuses = {"homing_status": 0, "enabled: ":False, "position": 0}
# camera_initialized = {"begin_initialization": False, "is_initialized": False}



# TRACKING PARAMETERS (n=1)
foundMagnetFlag = False
foundPostFlag = False
well = 0
CircularBufferSize = 3


def store_variables(d, **kwargs):
    for key, value in kwargs.items():
        if key in d:
            d[key] = value
    return d

def extract_variables(d, keys):
    all_values = []
    for key in keys:
        if key in d:
            all_values.append(d[key])
    return tuple(all_values)


centroids           = blob_centroids[well]
magnet_parameters   = magnet_tracking_parameters[well]
post_parameters     = post_tracking_parameters[well]
tissue_param        = tissue_parameters[well]
motor_status        = motor_statuses[well]
camera_inits        = camera_initialized[well]

tracker = PostAndMagnetTracker(
    (magnet_parameters['threshold'], (0, 10)),       # thresholds
    (magnet_parameters['area'],      (3000, 10000)), # area
    ROI_post, ROI_magnet,                                               # ROI
    CircularBufferSize=CircularBufferSize)


while True:
    clock.tick()
    tracker.tic = utime.ticks_ms()-t0
    img = sensor.snapshot()
    if uart.simulationMode == False:
        temp, command, info = uart.read()
        if command=='INIT': # INITIALIZE TRACKER
            well = temp
            camera_inits['begin_initialization'] = True

        elif command == 'CHANGE': # CHANGE WELL
            # Store current variables into dictionaries
            blob_centroids[well]                = centroids
            magnet_tracking_parameters[well]    = magnet_parameters
            post_tracking_parameters[well]      = post_parameters
            tissue_parameters[well]             = tissue_param
            motor_statuses[well]                = motor_status
            camera_initialized[well]            = camera_inits


            # change well, get dicts
            well = temp
            centroids           = blob_centroids[well]
            magnet_parameters   = magnet_tracking_parameters[well]
            post_parameters     = post_tracking_parameters[well]
            tissue_param        = tissue_parameters[well]
            motor_status        = motor_statuses[well]
            camera_inits        = camera_initialized[well]

            # update from stored variables
            area, extent, aspectratio, thresh = extract_variables(magnet_parameters,
                ('area','extent','aspectratio','threshold'))
            centroid_m, centroid_p = extract_variables(centroids,
                ('magnet_centroid','post_centroid'))

        elif command == 'POSTMANUAL': # CHANGING MANUAL POST
            post_parameters["manual_flag"] = True # force to be true
            centroids['passive_post_centroid'] = (int(info[0]), int(info[1]))
            centroids['post_centroid']         = (int(info[0]), int(info[1]))


    else:
        if camera_inits["is_initialized"] == False: # if not currently initialized
            camera_inits["begin_initialization"] == True # trigger initialization
        else:
            camera_inits["begin_initialization"] == False # hold low


    stop = pin7.value()
    if uart.simulationMode == True:
        stop = False
        if camera_inits["is_initialized"] == False:
            camera_inits["begin_initialization"] = True

    if stop:
        toc = utime.ticks_ms() - t0
        if toc > tracker.tic:
            frame_rate = 1000. / (toc-tracker.tic)

    if post_parameters['manual_flag']:
        img.draw_circle(centroids['post_centroid'][0], centroids['post_centroid'][1],
            10,color=255, fill=True)

    # INITIALIZE CAMERA
    if camera_inits["begin_initialization"]:

        img.draw_string(25,80, "CAMERA INITIALIZING", [255,0,0], 4, 1, 1, True, 0)
        utime.sleep_us(1)
        camera_inits["begin_initialization"] = False # FLIP LOW

        tracker.reset() # reset tracker

        stats_m = locate_magnet(img, magnet_tracking_parameters=magnet_parameters)
        stats_p = locate_post(img, post_tracking_parameters=post_parameters)

        if len(stats_m) > 0:
            # pick best stats_m
            stats_m = [stats_m[0]]
        else:
            extent = 0.6
            aspectratio = (1,3)
            k = 0
            for i in range(3):
                thresh, area = determineThresholds(img, magnet_or_post='magnet', tracking_parameters=magnet_parameters)
                if thresh == None:
                    # widen search parameters, try again
                    extent = extent - 0.1
                    aspectratio = (aspectratio[0]-0.2, aspectratio[1]+0.5)

                else: # thresh is not None, therefore found threshold
                    # update dictionary
                    magnet_parameters = store_variables(magnet_parameters,
                        extent=extent,
                        aspectratio=aspectratio,
                        threshold=thresh)

                    stats_m = locate_magnet(img, magnet_tracking_parameters=magnet_parameters)
                    # choose best
                    stats_m = [stats_m[0]]
                    break

            if thresh == None:
                # DID NOT FIND MAGNET
                print("FAILED INITIALIZATION")
                camera_inits['is_initialized'] = False


        # ASSUMPTION IS THAT stats_m IS VALID
        if len(stats_m) == 0:
            area = area
            extent = 0.6
            aspectratio = (1,3)


            camera_inits["begin_initialization"] = False
            camera_inits["is_initialized"] = False
            continue
        else:
            area = stats_m[0].area()
            extent = stats_m[0].extent()
            aspectratio = float(stats_m[0].w()) / float(stats_m[0].h())

            magnet_parameters = store_variables(magnet_parameters,
                area=(int(area*0.8), int(area*1.2)),
                extent=extent,
                aspectratio=(aspectratio * 0.8, aspectratio*1.2)
                )

            centroids = store_variables(centroids,
                passive_magnet_centroid =np.array((stats_m[0].cx(), stats_m[0].cy())),
                magnet_centroid         =np.array((stats_m[0].cx(), stats_m[0].cy())),
                )

            tracker.initPassive(stats_m, stats_p, post_tracking_parameters=post_parameters)

            tissue_param['passive_length'] = tracker.dist_neutral

            utime.sleep_ms(100)

            img = sensor.snapshot()

            stats_m = locate_magnet(img, magnet_tracking_parameters=magnet_parameters)
            stats_p = locate_post(img, post_tracking_parameters=post_parameters)

            stats_m, foundMagnetFlag = stats_check(stats_m, magnet_parameters, centroids)
            stats_p, foundPostFlag = stats_check(stats_p, post_parameters, centroids)

            centroid_m = np.array((stats_m[0].cx(), stats_m[0].cy()))
            centroid_p = np.array((stats_p[0].cx(), stats_p[0].cy()))


            value = tracker.processImage(img, PostAndMagnetTracker.computeStretchFrequency, centroid_m, centroid_p)

            stretch = value[0][-1]
            if stretch < -0.25 or stretch > 0.25:
                camera_inits['begin_initialization'] = True
                camera_inits['is_initialized'] = False
            else:
                camera_inits['is_initialized'] = True

            backup_magnet_parameters = magnet_parameters.copy()
            uart.send_over_serial(-43,
                [   tracker.tic,
                    well,
                    tissue_param['passive_length'],
                    magnet_parameters['threshold'],
                    post_parameters['threshold'],
                    centroids['passive_post_centroid'],
                    centroids['passive_magnet_centroid']
                ],
                    print_flag=True)
            continue

    if camera_inits['is_initialized']: # process, since it's already initialized
        stats_m = locate_magnet(img, magnet_tracking_parameters=magnet_parameters)
        stats_p = locate_post(img, post_tracking_parameters=post_parameters)
        #print(stats_m)
        stats_m, foundMagnetFlag = stats_check(stats_m, magnet_parameters, centroids)
        stats_p, foundPostFlag = stats_check(stats_p, post_parameters, centroids)

        found_backup_magnet_flag = False
        if foundMagnetFlag==False:
            #print('in backup')
            backup_magnet_parameters['roi'] = (285, 285+356, 81, 81+309)
            backup_magnet_parameters['extent'] = 0.6
            backup_magnet_parameters['aspectratio'] = (0.8, 2)
            backup_magnet_parameters['area'] = (1500, 4000)
            stats_m_backup = locate_magnet(img, magnet_tracking_parameters=backup_magnet_parameters)
            stats_m_backup, found_backup_magnet_flag = stats_check(stats_m_backup, backup_magnet_parameters, centroids)
            if found_backup_magnet_flag:
                #print('found a backup')
                #print(stats_m_backup)
                stats_m = stats_m_backup
                foundMagnetFlag = True

        #print(stats_m)
        centroid_m = np.array((stats_m[0].cx(), stats_m[0].cy()))
        centroid_p = np.array((stats_p[0].cx(), stats_p[0].cy()))


        value = tracker.processImage(img, PostAndMagnetTracker.computeStretchFrequency, centroid_m, centroid_p)

        ret, rotated = tracker.showTrackingOscilloscope(img, value, well, centroids, magnet_parameters)

        if ret != 0:
            continue

        n = len(value[0])
        if n > 0:
            stretch = value[0][-1]
            max_stretch = value[2]
        else:
            stretch = 0
            max_stretch = []

        #foundMagnetFlag = True
        if foundMagnetFlag:
            # update based on new magnet
            # get area, centroid, extent, aspectratio

            c_area, c_extent, c_aspectratio, c_threshold, c_roi = extract_variables(magnet_parameters,
                ('area','extent','aspectratio','threshold','roi'))

            area        = stats_m[0].area()
            centroid_m  = stats_m[0].cx(), stats_m[0].cy()
            extent      = stats_m[0].extent()
            aspectratio = float(stats_m[0].w()) / float(stats_m[0].h())

            # area around centroid
            centroid_box = [ centroid_m[0] - int(tracker.sizeStats/2),
                    centroid_m[1] - int(tracker.sizeStats/2),
                    tracker.sizeStats,
                    tracker.sizeStats]
            centroid_area = img.get_statistics(roi=centroid_box)

            # new region of interest
            mag_roi = ( stats_m[0].cx()-int(tracker.sizeROIy/2),
                        stats_m[0].cx()+int(tracker.sizeROIy/2),
                        stats_m[0].cy()-int(tracker.sizeROIx/2),
                        stats_m[0].cy()+int(tracker.sizeROIx/2))

            magnet_outline = stats_m[0].rect()
            img.draw_rectangle( mag_roi[0],
                                mag_roi[2],
                                mag_roi[1]-mag_roi[0],
                                mag_roi[3]-mag_roi[2],
                color=255, thickness=2)
            img.draw_rectangle( magnet_outline[0],
                                magnet_outline[1],
                                magnet_outline[2],
                                magnet_outline[3],
                color=155, thickness=2)

            # new threshold as median + 3*std
            # attempting to avoid pixel value outliers that may be present in the center of the magnet
            calculated_thresh = int(centroid_area.max()*1.3)
            if      calculated_thresh > c_threshold[1] + 1: threshold = c_threshold[1] + 1
            elif    calculated_thresh < c_threshold[1] - 1: threshold = c_threshold[1] - 1
            else:                                           threshold = c_threshold[1]

            #print(centroid_area)
            # store all newly calculated parameters
            magnet_parameters = store_variables(magnet_parameters,
                #area=(int(area*0.8), int(area*1.2)),
                extent=extent*0.9,
                aspectratio=(aspectratio*0.8, aspectratio*1.2),
                #threshold=(0,int(centroid_area.median() + 3*centroid_area.stdev())),
                threshold=(0,threshold),
                roi=mag_roi)

            if found_backup_magnet_flag:
                print("{:17} ({}) ".format('Found backup: ', round(frame_rate,2)), end='')
                uart.send_over_serial(-36,
                [   tracker.tic,
                    round(stretch, 1),
                    tracker.maxTracker.foundMax,
                    tracker.maxTracker.maxes.signal(),
                    foundMagnetFlag,
                    found_backup_magnet_flag
                ], print_flag = False)
            else:
                print("{:17} ({}) ".format('Found magnet: ', round(frame_rate,2)), end='')
                uart.send_over_serial(-35,
                [   tracker.tic,
                    round(stretch, 1),
                    tracker.maxTracker.foundMax,
                    tracker.maxTracker.maxes.signal(),
                    foundMagnetFlag,
                    found_backup_magnet_flag
                ], print_flag = False)


            [print("{}: {}".format(key, value), end='') for key, value in magnet_parameters.items()]
            print('')
        else:
            print("{:17} ({}) ".format('No magnet: ', round(frame_rate,2)), end='')
            [print("{}: {}, ".format(key, value), end='') for key, value in magnet_parameters.items()]
            uart.send_over_serial(-37,
                [   tracker.tic,
                    round(stretch, 1),
                    tracker.maxTracker.foundMax,
                    tracker.maxTracker.maxes.signal(),
                    foundMagnetFlag,
                    found_backup_magnet_flag
                ], print_flag = False)
            print('')

        toc = utime.ticks_ms() - t0
        if toc > tracker.tic:
            frame_rate = 1000. / (toc-tracker.tic)
            tracker.setFps(frame_rate)

        
        tracker.maxTracker.foundMax = 0
        continue

    toc = utime.ticks_ms() - t0
    if toc > tracker.tic:
        frame_rate = 1000. / (toc-tracker.tic)
        tracker.setFps(frame_rate)
        print("{}, ".format(frame_rate), end='')
        uart.send_over_serial(-47, [tracker.tic], print_flag = True)




