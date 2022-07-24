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
uart = UART(3, 9600, timeout = 0)
uart.init(9600, timeout = 0)

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


def show_stretch_cytostretcher_MV(centroid_magnet=(254, 376),
                               centroid_post=(259, 213),
                               thresh_range=((0, 40), (20, 60)),
                               area_range=((0, 1500), (3000, 10000)),
                               outfile=None):
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.VGA)
    sensor.skip_frames(time = 2000)
    sensor.set_auto_gain(False) # must be turned off for color tracking
    sensor.set_auto_whitebal(False) # must be turned off for color tracking
    clock = time.clock()

    fps = 15  # Target FPS
    nframes = 30*fps  # Process 30 seconds worth
    print(sensor.height())
    ROI_post	    = (	0,	250,			160,	320) # x, y, w, h
    ROI_magnet	    = (    250,	sensor.width(),	160,	320)
    
    tracker = cb.PostAndMagnetTracker(fps, nframes, thresh_range, area_range, roi_post = ROI_post, roi_magnet = ROI_magnet)

    if outfile is not None:
        output = mpeg.Mjpeg(outfile)
    else:
        output = None

    print("\nmilliseconds\tstretch_percent\tbeat_freq\tlast_max_stretch\tfps")  # tab-delimited header
    frame_rate = 0
    t0 = utime.ticks_ms()
    running = True
    starting_stretch = 0
    starting_freq = 0
    starting_max_stretch = 0
    value = ([starting_stretch], [starting_freq], [starting_max_stretch])
    centroid_m = (254, 376)
    centroid_p = (259, 213)
    milliseconds = 0
    time_of_max =  0

    plotting_parameters = (centroid_m, centroid_p, milliseconds, time_of_max, value)
    #for i in range(0, nframes):
    while (True):
        val = uart.readline()
        if val == None:
            pass
        else:
            val = val[:-1].decode('utf-8')
        print(val)
        tic = utime.ticks_ms() - t0
        # clock.tick()
        img = sensor.snapshot()

        # Query the toggle button (pin7)
        on = pin7.value()
        #print(on)
        if on: # stop running tracker
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
        if not running:
            continue;
        
        ret, rotated, value, plotting_parameters = tracker.processImage(img, tic, value, plotting_parameters, cb.PostAndMagnetTracker.computeStretchFrequency)
        #value = ((0),(0),(0))
        ret = 0
        if ret != 0:
            continue
        if output is not None:
            output.add_frame(img)

        toc = utime.ticks_ms() - t0
        if (toc > tic):
            frame_rate = 1000.0/(toc-tic)
            tracker.setFps(frame_rate)  # Used to compute beat_frequency
            #print(frame_rate)

        # For now display tracking info on console.
        if len(value[0]) > 0 and len(value[1]) > 0:
            #print("%.0f\t%.1f\t%.1f\t%.1f\t%.2f" % (tic, value[0][-1], value[1][-1], value[2], frame_rate) )  # stretch, beat_freq, max_stretch, fps
            t = uart.write("%.0f&%.1f&%.1f&%.1f&#\n" % (tic, value[0][-1], value[1][-1], value[2]) ) 
        elif len(value[0]) > 0 and len(value[1]) == 0:
            #print("%.0f\t%.1f\tNaN\tNaN\t%.2f" % (tic, value[0][-1], frame_rate) )  # stretch, beat_freq, max_stretch, fps
            uart.write("%.0f&%.1f&NaN&NaN&#\n" % (tic, value[0][-1]) )  # stretch, beat_freq, max_stretch, fps
        elif len(value[0]) == 0 and len(value[1]) >= 0:
            #print("%.0f\tNaN\t%.1f\tNaN\t%.2f" % (tic, value[1][-1], frame_rate) )  # stretch, beat_freq, max_stretch, fps
            uart.write("%.0f&NaN&%.1f&NaN&#\n" % (tic, value[1][-1]) )  # stretch, beat_freq, max_stretch, fps
        else:
            #print("%.0f\tNaN\tNaN\tNaN\t%.2f" % (tic, frames_per_second) )  # stretch, beat_freq, fps
            uart.write("%.0f&NaN&NaN&NaN&#\n" % (tic) )  # stretch, beat_freq, fps




show_stretch_cytostretcher_MV(outfile=None)  # Use defaults
