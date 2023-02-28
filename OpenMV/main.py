import sensor, image, time, math, mjpeg, omv, utime
from CuriBio_MV import *
from pyb import Pin, LED, UART
from ulab import numpy as np


sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.VGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(True)
#sensor.set_auto_exposure(False, 5000)
sensor.skip_frames(time = 1000)


extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.GRAYSCALE)
extra_fb.replace(sensor.snapshot())

pin7 = Pin('P7', Pin.IN, Pin.PULL_UP)

uart = Serial(3, 115200, timeout=0)
clock = time.clock()

uart.simulationMode = False

# INITIAL VALUES
ROI_post = (0, 250, 160, 320)
ROI_magnet = [  (250, 600, 150, 250),
                (250, 600, 125, 225),
                (250, 600, 125, 225),
                (250, 600, 100, 200)]

frame_rate = 0
t0 = utime.ticks_ms()

stretch, freq, max_stretch = [0],[0],[0]
value = (stretch, freq, max_stretch)

draw_outlines = True
# ROW PARAMTERS (n=4)
row_labels      = ['D',     'C',    'B',    'A']

blob_centroids = [  {"magnet_centroid":(0,0), "post_centroid": (53,237), "passive_magnet_centroid": (0,0), "passive_post_centroid": (53,237)},
                    {"magnet_centroid":(0,0), "post_centroid": (53,237), "passive_magnet_centroid": (0,0), "passive_post_centroid": (53,237)},
                    {"magnet_centroid":(0,0), "post_centroid": (53,237), "passive_magnet_centroid": (0,0), "passive_post_centroid": (53,237)},
                    {"magnet_centroid":(0,0), "post_centroid": (53,237), "passive_magnet_centroid": (0,0), "passive_post_centroid": (53,237)}]

magnet_tracking_parameters = [  {'area':(1000,4000),'extent':0.6,'aspectratio':(0.5,3),'threshold':(0,55), 'roi':ROI_magnet[0]},
                                {'area':(1000,4000),'extent':0.6,'aspectratio':(0.5,3),'threshold':(0,40), 'roi':ROI_magnet[1]},
                                {'area':(1000,4000),'extent':0.6,'aspectratio':(0.5,3),'threshold':(0,40), 'roi':ROI_magnet[2]},
                                {'area':(1000,4000),'extent':0.6,'aspectratio':(0.5,3),'threshold':(0,40), 'roi':ROI_magnet[3]}]

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

actual_parameters = {   'centroid':(0,0),
                        'extent':0.6,
                        'aspectratio':1.2,
                        'area':1500}

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
        if len(command) > 0:
            print(command, end=': ')
        if command=='INIT': # INITIALIZE TRACKER
            well = temp
            camera_inits['begin_initialization'] = True

        elif command == 'CHANGE': # CHANGE WELL
            # Store current variables into dictionaries
            tissue_param['passive_length']      = tracker.dist_neutral
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


            tracker.dist_neutral = tissue_param['passive_length']
            # update from stored variables
            area, extent, aspectratio, thresh = extract_variables(magnet_parameters,
                ('area','extent','aspectratio','threshold'))
            centroid_m, centroid_p = extract_variables(centroids,
                ('magnet_centroid','post_centroid'))

        elif command == 'POSTMANUAL': # CHANGING MANUAL POST
            post_parameters["manual_flag"] = True # force to be true
            centroids['passive_post_centroid'] = (int(info[0]), int(info[1]))
            centroids['post_centroid']         = (int(info[0]), int(info[1]))
            post_parameters['post_manual']     = (int(info[0]), int(info[1]))


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
        continue

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
                img = sensor.snapshot()
                thresh, area = determineThresholds(img, magnet_or_post='magnet', tracking_parameters=magnet_parameters, visualize=True)
                if thresh == None:
                    # widen search parameters, try again
                    extent = extent - 0.1
                    aspectratio = (aspectratio[0]-0.2, aspectratio[1]+0.5)

                else: # thresh is not None, therefore found threshold
                    # update dictionary]
                    img = sensor.snapshot()
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


        print(len(stats_m))
        if len(stats_m) == 0:
            area = area
            extent = 0.6
            aspectratio = (1,3)


            camera_inits["begin_initialization"] = False
            camera_inits["is_initialized"] = False
            continue
        else:# ASSUMPTION IS THAT stats_m IS VALID
            area = stats_m[0].area()
            extent = stats_m[0].extent()
            aspectratio = float(stats_m[0].w()) / float(stats_m[0].h())

            magnet_parameters = store_variables(magnet_parameters,
                area=(int(area*0.8), int(area*1.2)),
                extent=extent,
                aspectratio=(aspectratio * 0.8, aspectratio*1.2)
                )

            centroids = store_variables(centroids,
                passive_magnet_centroid =(stats_m[0].cx(), stats_m[0].cy()),
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
        #print('ALGO')
        #utime.sleep_ms(500)
        # perform algorithmic changes to parameters
        if type(stats_m[0]) == type(Centroid((0,0))):
            threshold = magnet_parameters['threshold']
            went_through_param_algorithm = False

        else:
            went_through_param_algorithm = True
            c_area, c_extent, c_aspectratio, c_threshold, c_roi = extract_variables(magnet_parameters,
                    ('area','extent','aspectratio','threshold','roi'))


            area, extent, aspectratio, centroid_m = get_parameters_from_stats(stats_m[0])

            actual_parameters = store_variables(actual_parameters,
                extent=extent,
                aspectratio=aspectratio,
                centroid=centroid_m,
                area=area
                )
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

            smudgeFactor = 1.1
            threshold, direction = calculate_threshold(centroid_area, smudgeFactor, c_threshold)
            # attempting to avoid pixel value outliers that may be present in the center of the magnet

            #utime.sleep_ms(500)

            # find magnet, using new parameters
            print(actual_parameters)
            magnet_parameters = store_variables(magnet_parameters,
                area=(int(area*0.5), int(area*2.5)),
                extent=round(extent*0.7,2),
                aspectratio=(round(aspectratio*0.5,2), round(aspectratio*1.5,2)),
                #threshold=(0,int(centroid_area.median() + 3*centroid_area.stdev())),
                threshold=(0,threshold),
                roi=mag_roi)




        img.binary([(0,magnet_parameters['threshold'][1])], True, True)
        #utime.sleep_ms(2000)
        stats_m = locate_magnet(img, thresh_range=(1,magnet_parameters['threshold'][1]),magnet_tracking_parameters=magnet_parameters)
        stats_p = locate_post(img, post_tracking_parameters=post_parameters)

        # verify that a magnet was found
        stats_m, foundMagnetFlag = stats_check(stats_m, magnet_parameters, centroids)
        stats_p, foundPostFlag = stats_check(stats_p, post_parameters, centroids)
        #print(stats_m)

        found_backup_magnet_flag = False
        #utime.sleep_ms(500)
        if foundMagnetFlag==False:
            #print('in backup')
            backup_magnet_parameters['roi'] = ROI_magnet[well]
            backup_magnet_parameters['extent'] = 0.6
            backup_magnet_parameters['aspectratio'] = (0.2, 5)
            backup_magnet_parameters['area'] = (1500, 4000)
            #print('BACKUP')
            stats_m_backup = locate_magnet(img, magnet_tracking_parameters=backup_magnet_parameters)
            stats_m_backup, found_backup_magnet_flag = stats_check(stats_m_backup, backup_magnet_parameters, centroids)
            if found_backup_magnet_flag:
                #print('found a backup')
                #print(stats_m_backup)
                stats_m = stats_m_backup
                foundMagnetFlag = True

        if went_through_param_algorithm:
            if draw_outlines == True:
                img.draw_rectangle( mag_roi[0],
                                    mag_roi[2],
                                    mag_roi[1]-mag_roi[0],
                                    mag_roi[3]-mag_roi[2],
                    color=[155,0,0], thickness=2)
                img.draw_rectangle( magnet_outline[0],
                                    magnet_outline[1],
                                    magnet_outline[2],
                                    magnet_outline[3],
                    color=[0,155,0], thickness=2)

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
            if found_backup_magnet_flag:
                magnet_or_backup = 'backup'
                magic = -36

            else:
                magnet_or_backup = 'magnet'
                magic = -35
            #img.to_rgb565()

        else:
            magnet_or_backup = 'none'
            if draw_outlines == True:
                img.draw_rectangle( magnet_parameters['roi'][0],
                                    magnet_parameters['roi'][2],
                                    magnet_parameters['roi'][1]-magnet_parameters['roi'][0],
                                    magnet_parameters['roi'][3]-magnet_parameters['roi'][2],
                    color=[155,0,0], thickness=2)
            magic = -37


        print("Found {:17} ({}) ".format(magnet_or_backup, round(frame_rate,1)), end='')
        [print("{}, ".format(value), end='') for key, value in magnet_parameters.items()]
        uart.send_over_serial(magic,
            [ tracker.tic,
              round(stretch, 1),
              tracker.maxTracker.foundMax,
              tracker.maxTracker.maxes.signal(),
              foundMagnetFlag,
              found_backup_magnet_flag
              ], print_flag = True)
        #[print("{}: {}, ".format(key, value), end='') for key, value in actual_parameters.items()]
        #print(tracker.dist_neutral)
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
        print("{},{}, ".format(round(frame_rate,1), stop), end='')
        [print("{}: {}, ".format(key,value), end='') for key, value in magnet_parameters.items()]

        uart.send_over_serial(-47, [tracker.tic], print_flag = True)
