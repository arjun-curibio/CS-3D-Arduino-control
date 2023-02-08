# CuriBio_MV module
#
# Version: 2022_07_15
import sensor, image, time, math, utime
from pyb import Pin, LED, UART
from ulab import numpy as np

toggleButton = Pin('P7', Pin.IN, Pin.PULL_UP)  # Connected to pin 7
green_led = LED(2)
red_led   = LED(1)

class Serial:
    def __init__(self, pin, baudrate, timeout=25):
        if pin == -1:
            self.simulationMode = True
        else:
            self.simulationMode = False
            self.uart = UART(pin, baudrate, timeout=timeout)
            self.uart.init(baudrate, timeout=timeout)
            #self.uart.CameraSpeed
    
    def read(self):
        val = ''
        current_well, command, info = 0, '', []
        #print('reading')
        val = self.uart.read()
        print(val)
        if val is not None:
            if len(val) > 4 and len(val) < 200:
                #print(len(val))
                
                try: 
                    val = val.decode('utf-8')
                    info = val.split('#')[0].split('&')
                    current_well = int(info.pop(0))
                    command = info.pop(0)
                except:
                    pass
        return current_well, command, info
    
    def write(self, string):
        if self.simulationMode == False:
            self.uart.write(string+'\n')

    def send_over_serial(self, magic, info, print_flag=True):
            info = [str(magic)]+info
            outstring = "#".join(info)
            self.uart.write(outstring+'\n')
            if print_flag:
                print(outstring)
        

# import random

class Centroid: # Class to hide a centroid inside something that acts like a stats_p or stats_m
    def __init__(self, centroid):
        # centroid is (x,y)
        self.centroid = centroid
        self.radius = 50
        self.stats = [] # set empty for now
    def set_enclosing_circle(self, xyr):
        self.centroid = (xyr[0], xyr[1])
        self.radius = xyr[2]
    def cx(self):
        return self.centroid[0]
    def cy(self):
        return self.centroid[1]
    def pixels(self):
        if len(self.stats) > 0:
            return self.stats[0].area() # returning area
        else:
            return 0  # We can use this to skip over the default_post_centroid plotting
    def get_centroid(self):
        return self.centroid
    def get_enclosing_circle(self):
        return (self.centroid[0], self.centroid[1], self.radius)
    def set_stats(self, stats):
        self.stats = stats
    def get_stats(self):
        return self.stats

def find_first(x):
    # From Stackoverflow "Numpy:find first index of value fast"
    idx = x.view(bool).argmax() // x.itemsize
    return idx if x[idx] else -1

class CircularBuffer:
    # Track values, resetting the signal after it reaches the end
    def __init__(self, buflen):
        self.buffer = []
        self.buflen = buflen
        
        
        #self.buffer = np.zeros((buflen,))
        #self.buflen = buflen
        #self.count = 0
        #self.head = 0  # current end of buffer
        #self.bufstart = 0  # buffer epoch (cum index of signal[0])
    def append(self, v):
        self.buffer.append(v) # append to list
        if len(self.buffer) > self.buflen: # if the new list is too large
            self.buffer.pop(0) # remove the first number from the list
    def signal(self):
        return self.buffer
    def length(self):
        return len(self.buffer)
    def reset(self):
        self.head = 0
        self.count = 0
        self.bufstart = 0
class MaxTracker:
    """ Track the index of maximum values in a signal
    """

    def __init__(self, buflen, CircularBufferSize,n_hysteresis):
        self.tracker = CircularBuffer(buflen)  # List of v
        self.milliseconds = CircularBuffer(buflen) # List of t
        self.maxes = CircularBuffer(CircularBufferSize)  # List of last few v at max in signal
        self.max_times = CircularBuffer(CircularBufferSize) # List of last few t at max in signal
        
        
        self.max_signal = None # remember last max signal
        self.update_max = True

        self.last_max = float('-Inf') # Following max up
        self.last_tmax = 0

        self.last_min = float('Inf') # Following min down
        self.last_tmin = 0

        # Protect against out of range frequency
        max_freq = 6.0  # in Hz
        self.min_dt = 1000.0 / max_freq  # in milliseconds per cycle

        self.hysteresis = 1
        self.n_hysteresis = n_hysteresis
        self.count = -1  # Number of values processed
        self.hysteresis_signal = [] # Last 20 signal values (used to apply hysteresis-based max finding)
        
        self.avg_filter = 0
        self.direction = 0
        self.foundMax = 0
        self.v = 0
        self.countMaxes = 0
    
    
    def process(self, t, v) -> int:
        """ Process time and signal value while looking for maximums
        Pass in the values sequentially

         :param v: is the next value to process
         :return: +1 if maximum detected, -1 if minimum detected
                   or 0 otherwise
        """
        direction = 0
        self.direction = 0
        self.count += 1
        
        self.v = v
        # track Signal
        self.milliseconds.append(t)
        self.tracker.append(v)

        # Remember the last 20 points of v
        self.hysteresis_signal.append(v)
        if len(self.hysteresis_signal) > self.n_hysteresis:
            self.hysteresis_signal.pop(0)
        
        # Compute hysteresis from last 20 points
        # This could be replaced by a recursive formula
        if len(self.hysteresis_signal) > 3:
            std = np.std(self.hysteresis_signal)
            self.avg_filter = np.mean(self.hysteresis_signal)
            self.hysteresis = max(std, 2)  # Don't allow std to go below 0.5
            
        
        # Follow max up
        i = self.count
        if i > 4 and self.update_max and v > self.last_max:
            # Found a higher max
            self.last_max = v
            self.last_tmax = t

        # Follow min down
        if i > 4 and not self.update_max and v < self.last_min:
            # Found a lower min
            self.last_min = v
            self.last_tmin = t

        if self.update_max and v < (self.last_max - self.hysteresis) and t > self.last_tmin + self.min_dt:
            # Update hysteresis dynamically (but slowly)
            # if len(self.maxes) > 0:
            #     delta = self.last_max - self.last_min
            #     if delta > self.hysteresis * 4:
            #         self.hysteresis_signal.pop(-1)  # Remove spurious point
            #     std = np.std(self.hysteresis_signal)
            #     self.hysteresis = std/2  # 1/2 standard deviation
            #     self.hysteresis = (self.hysteresis + delta / 4) / 2
            # max_twitch = self.last_max

            # a new max verified so add to list of argmax's
            if len(self.maxes.signal()) == 0 or self.max_times.signal()[-1] != self.last_tmax:
                # print('max verified at t=', self.last_tmax)
                if self.last_max > 3: # make sure maximums are greater than 3 pixels
                    self.max_times.append(self.last_tmax)
                    self.maxes.append(round(self.last_max))
                    self.max_signal = round(self.last_max)  # Remember last maximum signal
                    self.foundMax = 1
                    self.countMaxes += 1
            self.last_min = self.last_max
            self.last_tmin = self.last_tmax
            self.update_max = False  # Now looking for min
            
            direction = +1  # max detected
            self.direction = 1

        elif not self.update_max and v > (self.last_min + self.hysteresis):
            # Update hysteresis dynamically (but slowly)
            # if len(self.maxes) > 0:
            #     delta = self.last_max - self.last_min
            #     if delta > self.hysteresis * 4:
            #         self.hysteresis_signal.pop(-1)  # Remove spurious point
            #     std = np.std(self.hysteresis_signal)
            #     self.hysteresis = std / 2  # 1/2 standard deviation
            #     self.hysteresis = (self.hysteresis + delta / 4) / 2
            # This is where we might update the passive deflection
            # if self.last_min < passive_deflection
            #     passive_deflection = self.last_min / spring_constant / microns_per_pixel
            #     passive_tension[i-1:i] = spring_constant * passive_deflection * microns_per_pixel
            self.last_max = self.last_min
            self.last_tmax = self.last_tmin
            self.update_max = True  # Now looking for max
            direction = -1  # min detected
            self.direction = -1

        return direction

    def signal(self):
        return self.tracker.signal()

    def signal_offset(self):
        return self.tracker.buf_start()

    def time(self):
        return self.milliseconds.signal()

    def maximums(self):
        # value at signal maxes
        return self.maxes.signal()

    def time_of_maximums(self):
        return self.max_times.signal()

    def last_max_signal(self):
        return self.max_signal

    def reset(self):
        self.tracker.reset()
        self.milliseconds.reset()
        self.maxes.reset()
        self.max_times.reset()
        self.update_max = False
        self.last_max = float('-Inf')
        self.last_tmax = 0
        self.last_min = float('Inf')
        self.last_tmin = 0
        self.hysteresis = 10
        self.count = -1
        

class PostAndMagnetTracker:

    # added more initialization parameters
    def __init__(self, thresh_range, area_range, roi_post, roi_magnet, roi=(72, 420, 199, 270), CircularBufferSize=10):
        """ PostAndMagnetTracker Constructor

        tracker = PostAndMagnetTracker(thresh_range,area_range)

        :param thresh_range: is a 2 element list of (min,max) threshold ranges for magnet and post
        :param area_range: is a 2 element list of (min,max) area ranges for magnet and post
        :param roi is a 4 element tuple (xmin,xmax,ymin,ymax)
        """
        frames_to_plot = 200
        n_hysteresis = 50
        self.maxTracker = MaxTracker(frames_to_plot, CircularBufferSize, n_hysteresis)
        self.CircularBufferSize = CircularBufferSize
        self.n_hysteresis = n_hysteresis
        self.thresh_range = thresh_range
        self.area_range = area_range
        self.roi_post = roi_post
        self.roi_magnet = roi_magnet
        self.extent = 0.6
        self.aspectratio = (0.5, 2.0)
        self.circularity = 0.5
        self.roi = (0,sensor.width(), 0, sensor.height())

        self.tic = utime.ticks_ms()

        self.wait_until = 0
        # Determine good x-axis ticks

        self.font_scale = 3
        self.image_scale = 3
        self.spring_constant = 0.159
        
        self.centroid_m = None
        self.centroid_p = None
        self.automatic_thresh = False  # Automatically determine the thresholds
        self.stats_m = None
        self.centroid_p = [0,0]
        self.value = [],[],[]
        # Lazily initialized when first image is processed
        self.passive_deflection = None  # deflection in pixels
        self.dist_neutral = None  # distance in pixels
        self.microns_per_pixel = None
        self.post_centroid = None
        self.InitializedPostCentroid = None
        self.twitch_deflection = 0

        # Lazily initialized title and oscilloscope areas
        self.zoom_rect = None
        self.title_rect = None

        self.last_max_array = []
        self.sizeROIx = 80
        self.sizeROIy = 150
        self.sizeStats = 20
        self.smudgeFactor = 1.25

    def setFps(self, fps):
        self.fps = fps

    def reset(self): # reset tracker initialization
        self.passive_deflection = None
        self.dist_neutral = None
        self.maxTracker = MaxTracker(200, self.CircularBufferSize, self.n_hysteresis)
        self.roi_magnet = (250, 600, 160, 320)
        self.roi_post = (0, 250, 160, 320)
        

    # This should be private
    def initPassive(self, stats_m, stats_p, postManualFlag=True, postManual=(57,237), post_tracking_parameters=None):
        if post_tracking_parameters is not None:
            postManualFlag  = post_tracking_parameters['manual_flag']
            postManual      = post_tracking_parameters['post_manual']
        if postManualFlag:
            centroid_p = postManual
        elif stats_m == None or stats_p == None:
            return (0,0), (0,0)
        
        if len(stats_m) == 0:
            centroid_m = np.array((0,0))
        else:
            centroid_m = np.array((stats_m[0].cx(), stats_m[0].cy()))
        
        if postManualFlag:
            centroid_p = np.array(postManual)
        else:
            if len(stats_p) == 0:
                centroid_p = np.array(postManual)
            else:
                centroid_p = np.array((stats_p[0].cx(), stats_p[0].cy()))

        
        dx = centroid_m[0] - centroid_p[0]
        dy = centroid_m[1] - centroid_p[1]

        # Determine the passive deflection
        dist_current = np.linalg.norm(centroid_m - centroid_p)

        if self.passive_deflection is not None:
            # Follow max distance up
            self.dist_neutral = max(self.dist_neutral, dist_current)
            self.passive_deflection = self.dist_neutral - dist_current  # Distance between posts during relaxation of issue
        else:
            # dist_current = distance.euclidean(centroid_m, centroid_p)  # distance between post and magnet
            self.dist_neutral = dist_current
            self.passive_deflection = self.dist_neutral - dist_current  # Distance between posts
            # Determine location of title rect
            self.title_rect = (0, 0, sensor.width(), 80)
            
        return dist_current # added return values

    def steps(self, signal, pos, signal_offset, n):
        # Create n-length step function using signal that changes at positions pos
        #  We expect len(signal) == len(pos)-1
        y = np.zeros(n)
        trace = signal - np.concatenate((np.array([0,]), signal[0:-1]), axis=0)
        # print('trace=', trace, '   n=', n)

        # Insert impulse train into y at positions pos
        for i in range(0,len(trace)):
            # Take into account the circularity of the signal buffer
            if pos[i+1] >= signal_offset:
                y[int(pos[i+1]-signal_offset)] = trace[i]
            else:
                y[0] = trace[i]

        # y = np.cumsum(y) inline implementation
        s = 0.0
        for i in range(0,n):
            s = s + y[i]
            y[i] = s

        return y

    def computeStretchFrequency(self, tracker):
        # Inputs:
        #    tracker: MaxTracker object
        # Outputs:
        #    stretch_percent, frequency, last_max_percent
        stretch_percent = np.array(tracker.signal()) / self.dist_neutral * 100  # percent of the neutral displacement
 #       maximums = tracker.maximums()
        time_of_max = tracker.time_of_maximums() # in milliseconds
        if time_of_max is not None:
            if len(time_of_max) > 1:
                last_max_percent = [tracker.maxes.signal()[-1] / self.dist_neutral * 100]
                
                # Compute beat frequency from signal
                
                # Estimate frequency of maximums
                f = 1000.0 / np.diff(np.array(time_of_max))
                # f = np.zeros((len(maximums)-1))
                # for m in range(1,len(maximums)):
                #     m0 = int(maximums[m-1])
                #     m1 = int(maximums[m])
                #     f[m-1] = 1000.0 / (t[m1] - t[m0])

                # Estimate frequency of maximums
                # f = self.fps / np.diff(maximums)
                return stretch_percent, f, last_max_percent
            else:
                return stretch_percent, [], []
        else:
            return stretch_percent, [], []
    def processImage(self, img, func, centroid_m, centroid_p):
        dist_current = np.linalg.norm(centroid_m - centroid_p)
        twitch_deflection = dist_current - self.dist_neutral
        self.twitch_deflection = twitch_deflection
        
        self.maxTracker.process(self.tic, twitch_deflection)
        value = func(self, self.maxTracker)  # returns 

        return value
    def showTrackingOscilloscope(self, img, value, current_well, blob_centroids, tracking_parameters):
        # Highlight the location of the magnet and post centroids
        # Show plot of value as a function of milliseconds
        # maximums is the same length as value[1]

        well_labels = ['D','C','B','A']

        width = sensor.width()
        height = sensor.height()

        # Block out title and oscilloscope areas
        img.draw_rectangle(self.title_rect, color=10, fill=True)
        # img.draw_rectangle(self.oscilloscope_rect, color=0, fill=True)

        # Indicate the axes area
        # img.draw_rectangle(self.axes_rect, color=255, thickness=2)

        # Draw centroid in image
        ret = self.showTracking(img, blob_centroids['magnet_centroid'], blob_centroids['post_centroid'])

        # Plot the signal like an oscilloscope = some function of twitch_deflection and max
        if len(value) > 0:
            stretch = value[0]  # From computeStretchFrequency
            beat_freq = value[1]  # From computeStretchFrequency
            max_stretch = value[2]

            
            if current_well < 4 and current_well >= 0:
                well_row = well_labels[current_well]
            else:
                well_row = 'Error'
            
            # Determine title
            title = "Row {}".format(well_row)
            title += " / {} s".format(round(self.tic/1000,1))
            title += "\nStretch: %4.2f%%" % stretch[-1]
            if len(max_stretch) == 0:
                title += " / Max Stretch: []"
                title += " / Thresh: {} ".format(tracking_parameters['threshold'])
            else:
                title += " / Max Stretch: %4.1f%%" % max_stretch[-1]
                title += " / Thresh: {} ".format(tracking_parameters['threshold'])
                
                title += " \nMax Stretch: {}".format(self.maxTracker.countMaxes)
            
            self.showTitle(img, title)

            # start = int(milliseconds[0])
            # self.showXTicks(img, (start, start + int(self.millisecs)), 255)

        #
        # rotated = img[rows][:, cols]
        # cv2.imshow("rotated", rotated
        # cv2.waitKey(20)

        return 0, img
    def showTitle(self, img, title):
        # Add title
        if title != "":
            position = (self.title_rect[0] + 1, self.title_rect[1] + 1)
            # print('title position=', position, 'title=', title)
            img.draw_string(position[0], position[1], title, scale=self.font_scale, color=255, mono_space=False)
    def showTracking(self, img, centroid_m, centroid_p):
        # Locate magnet and post
        width = sensor.width()
        height = sensor.height()
        
        img.draw_circle(int(centroid_m[0]), int(centroid_m[1]), 2, color=255, fill=True)  # marker
        # img.draw_circle(int(centroid_p[0]), int(centroid_p[1]), 10, color=127, fill=True)  # marker

        return 0

def nonzero(x):
    # Return positional argument of the nonzero elements
    nz = []
    for i in range(0,len(x)):
        if x[i] != 0:
            nz.append(i)

    return nz


def nonzero_runs(y):
    # Returns arrays or runs with run length.  Both could be empty (length 0)

    # Augment y with points before and after
    z = np.zeros((1,))
    yaug = np.concatenate((z, np.array(y), z), axis=0)

    # Turn yaug into a boolean
    yaug = np.array(yaug > 0, dtype=np.int16)  # .astype(int)

    # Find the beginning index of constant runs within boolean array y and associated runlen
    dy = np.diff(yaug)
    run_begin = nonzero(dy >= 1)
    run_end = nonzero(dy <= -1)
    # if run_begin is not None and run_end is not None:
    runlen = np.array(run_end) - np.array(run_begin)
    
    return run_begin, runlen


def getCentroid(d):
    return [d.cx(), d.cy()]
def getAspectRatio(d):
    return float(d.w()) / float(d.h())

def determineThresholds(img, area=((1000,3000),(3000,15000)), roi=(250, 600, 160, 320), magnet_or_post='magnet', visualize=False, tracking_parameters=None):
    r = range(0, 150)
    data = []
    for t in r:
        threshold = (0, t)
        if magnet_or_post == 'magnet':
            if tracking_parameters is not None:
                stats = locate_magnet(  img, threshold = (0,t), magnet_tracking_parameters=tracking_parameters)
            else:
                stats = locate_magnet(  img, (threshold, (0,0)), (area, (0,0)), roi, aspectratio = (1,3), extent=0.6)
        elif magnet_or_post == 'post':
            if tracking_parameters is not None:
                stats = locate_post(    img, threshold = (0,t), post_tracking_parameters=tracking_parameters)
            else:
                stats = locate_post(    img, ((0,0), threshold), ((0,0), area), roi, circularity = 0.5)
        data.append((t, len(stats)))
        

    # Extract data
    idx     = [x[0] for x in data]
    nblobs  = [x[1] for x in data]
    
    # Determine runs for magnet
    # Choose run with maximum length and then set the threshold in the middle of the run

    runs, runlen = nonzero_runs(nblobs)
    if len(runlen) > 0:
        runs = np.array([runs], dtype=np.uint16)
        runmax = np.argmax(np.array([runlen]))
        idx = runs[0][runmax]
        thresh = [0, int(r[idx] + runlen[runmax] // 2)]
    else:
        thresh = None
        idx = None
    
    return thresh, area

def FilterByArea(stats, area_range):
    good = []
    for props in stats:
        area = props.area()
        if area_range[0] <= area <= area_range[1]:
            good.append(props)

    return good
def FilterByExtent(stats, e):
    good = []
    for props in stats:
        extent = props.extent()
        if extent >= e:
            good.append(props)

    return good
def FilterByAspectRatio(stats, ar):
    good = []
    for props in stats:
        aspect_ratio = getAspectRatio(props)
        if aspect_ratio >= ar[0] and aspect_ratio <= ar[1]:
            good.append(props)

    return good
def FilterByCircularity(stats, circ, ar_range=(0.7, 1.4)):
    good = []
    for props in stats:
        # Expect circular blobs to be bounded by a "square" and high circularity
        circularity = props.roundness()
        aspect_ratio = props.h() / props.w()
        if circularity >= circ and ar_range[0] < aspect_ratio < ar_range[1]:
            good.append(props)

    return good
def FilterByROI(stats, roi_x, roi_y):
    good = []
    for props in stats:
        cx = props.cx()
        cy = props.cy()
        if (roi_x[0] <= cx <= roi_x[1]) and (roi_y[0] <= cy <= roi_y[1]):
            good.append(props)

    return good
def ExtentKey(x):
    return x.extent()
def CircularityKey(x):
    # Weighted cicularity
    circularity = x.roundness()
    return circularity
def OrderByExtent(stats):
    """ Return sorted array of statistics dictionaries sorted by Extent property
    """
    sorted_stats = sorted(stats, key=ExtentKey, reverse=True)
    return sorted_stats
def OrderByCircularity(stats):
    """ Return sorted array of statistics dictionaries sorted by Circularity property
    """
    sorted_stats = sorted(stats, key=CircularityKey, reverse=True)
    return sorted_stats

def locate_magnet(img, thresh_range=None, area_range=None, roi=None, aspectratio = None,  extent = None, magnet_tracking_parameters = None):
    # grab parameters from dictionary
    if magnet_tracking_parameters is not None:
        # only grab paramters if they're not function inputs
        if thresh_range is None:
            thresh_range    = magnet_tracking_parameters['threshold']
        if area_range is None:
            area_range      = magnet_tracking_parameters['area']
        if extent is None:
            extent          = magnet_tracking_parameters['extent']
        if aspectratio is None:
            aspectratio     = magnet_tracking_parameters['aspectratio']
        if roi is None:
            roi             = magnet_tracking_parameters['roi']
    
    stats_m = img.find_blobs( [thresh_range[0]], pixels_threshold=area_range[0], roi=(roi[0], roi[2], roi[1]-roi[0], roi[3]-roi[2]))
    
    # Filter results by area
    stats_m = FilterByArea(stats_m, area_range)
    #[print("{},{},{}".format(i.extent(), getAspectRatio(i), i.pixels())) for i in stats_m]
    
    #print("\n")
    # Filter by ROI
    # stats_m = FilterByROI(stats_m, roi_x, roi_y)

    # Filter by aspect ratio (expect tall approx 5:2 filter those below 0.9)
    stats_m = FilterByAspectRatio(stats_m, aspectratio)

    # Filter by extent (filter those below 0.6)
    stats_m = FilterByExtent(stats_m, extent)

    # Order by Extent
    stats_m = OrderByExtent(stats_m)

    return stats_m

def stats_check(stats, tracking_parameters, blob_centroids):
    if 'manual_flag' in tracking_parameters:
        # post
        if tracking_parameters['manual_flag']:
            return [stats[0]], False
        elif len(stats) > 0:
            return [stats[0]], True
        else:
            return [Centroid(blob_centroids['post_centroid'])], False
    else:
        if len(stats) > 0:
            return [stats[0]], True
        else:
            return [Centroid(blob_centroids['magnet_centroid'])], False

def locate_post(img, thresh_range=None, area_range=None, roi=None, circularity=None, post_manual_flag = True, post_manual = (57,237), post_tracking_parameters=None ):
    # grab parameters from dictionary
    if post_tracking_parameters is not None:
        # only grab paramters if they're not function inputs
        if thresh_range is None:
            thresh_range    = post_tracking_parameters['threshold']
        if area_range is None:
            area_range      = post_tracking_parameters['area']
        if circularity is None:
            circularity     = post_tracking_parameters['circularity']
        if roi is None:
            roi             = post_tracking_parameters['roi']
        
        # always grab these from dictionary
        post_manual_flag= post_tracking_parameters['manual_flag']
        post_manual     = post_tracking_parameters['post_manual']
    
    if post_manual_flag == True:
        return [Centroid(post_manual)]
    
    if thresh_range[1] == None:
        thresh_to_use = (0,15)
    else:
        thresh_to_use = thresh_range[1]
    stats_p = img.find_blobs( [thresh_to_use], pixels_threshold=area_range[0], roi=(roi[0], roi[2], roi[1]-roi[0], roi[3]-roi[2]), x_stride=5, y_stride=5)
    got_one = len(stats_p) > 0
    stats_p = FilterByArea(stats_p, area_range)
    stats_p = FilterByCircularity(stats_p, circularity)
    # Order by Circularity
    stats_p = OrderByCircularity(stats_p)

    return stats_p
