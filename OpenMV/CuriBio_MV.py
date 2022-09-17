# CuriBio_MV module
#
# Version: 2022_07_15
import sensor, image, time, math, utime
from pyb import Pin, LED, UART
from ulab import numpy as np

toggleButton = Pin('P7', Pin.IN, Pin.PULL_UP)  # Connected to pin 7
green_led = LED(2)
red_led   = LED(1)

# import random

class Serial:
    def __init__(self, pin, baudrate, timeout=25):
        if pin==-1:
            self.simulationMode=True
        else:
            self.simulationMode=False
            self.uart = UART(pin, baudrate, timeout=timeout)
            self.uart.init(baudrate, timeout=timeout)

    def read(self):
        val = ''
        current_well, command, info = 0,'',[]
        val = self.uart.read()
        #print(val)
        if val is not None:
            if len(val) > 4:
                val = val.decode('utf-8')
                info = val.split('#')[0].split('&')
                current_well = int(info.pop(0)) - 1
                command = info.pop(0)
        return current_well, command, info
    
    def write(self, string):
        self.uart.write(string+'\n')

    

def eye():
    return np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]])


def find_first(x):
    # From Stackoverflow "Numpy:find first index of value fast"
    idx = x.view(bool).argmax() // x.itemsize
    return idx if x[idx] else -1

class CircularBuffer:
    # Track values, resetting the signal after it reaches the end
    def __init__(self, buflen):
        self.buffer = np.zeros((buflen,))
        self.buflen = buflen
        self.count = 0
        self.head = 0  # current end of buffer
        self.bufstart = 0  # buffer epoch (cum index of signal[0])

    def length(self):
        if self.count < self.buflen:
            return self.head
        else:
            return self.buflen

    def last(self):
        return self.buffer[self.head-1]

    def maxlen(self):
        return self.buflen

    def buf_start(self):
        # Cumulative index of buf[0]
        return max(0, self.count - self.buflen)

    def append(self, v):
        if self.head == self.buflen:
            self.head = 0  # Wrap around
        self.buffer[self.head] = v
        self.head += 1
        self.count += 1

    def count(self):
        return self.count # Total count

    def signal(self) -> np.array:
        if self.count > self.buflen:
            first_part = self.buffer[self.head+1:]
            second_part = self.buffer[:self.head]
            return np.concatenate((first_part, second_part), axis=0)
        else:
            return self.buffer[:self.head]

# Here I am:  Need more of these methods updated to be used by the circular buffer

    def reset(self):
        self.head = 0
        self.count = 0
        self.bufstart = 0


class MaxTracker:
    """ Track the index of maximum values in a signal
    """

    def __init__(self, buflen):
        self.tracker = CircularBuffer(buflen)  # List of v
        self.milliseconds = CircularBuffer(buflen) # List of t
        self.maxes = CircularBuffer(20)  # List of last few v at max in signal
        self.max_times = CircularBuffer(20) # List of last few t at max in signal

        self.max_signal = None # remember last max signal
        self.update_max = False

        self.last_max = float('-Inf') # Following max up
        self.last_tmax = 0

        self.last_min = float('Inf') # Following min down
        self.last_tmin = 0

        # Protect against out of range frequency
        max_freq = 6.0  # in Hz
        self.min_dt = 1000.0 / max_freq  # in milliseconds per cycle

        self.hysteresis = 0.8
        self.count = -1  # Number of values processed
        self.last_20 = [] # Last 20 signal values (used to apply hysteresis-based max finding)

    def process_without_max(self, t, v) -> int:
        direction = 0
        self.count += 1
        self.milliseconds.append(t)
        self.tracker.append(v)
        return direction

    def process(self, t, v) -> int:
        """ Process time and signal value while looking for maximums
        Pass in the values sequentially

         :param v: is the next value to process
         :return: +1 if maximum detected, -1 if minimum detected
                   or 0 otherwise
        """
        direction = 0
        self.count += 1

        # track Signal
        self.milliseconds.append(t)
        self.tracker.append(v)

        # Remember the last 20 points of v
        if len(self.last_20) < 20:
            self.last_20.append(v)
        else:
            self.last_20.append(v)
            self.last_20.pop(0)

        # Compute hysteresis from last 20 points
        # This could be replaced by a recursive formula
        if len(self.last_20) > 3:
            std = np.std(self.last_20)
            self.hysteresis = max(std, 0.5)  # Don't allow std to go below 0.5

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

        if self.update_max and v < self.last_max - self.hysteresis and t > self.last_tmin + self.min_dt:
            # Update hysteresis dynamically (but slowly)
            # if len(self.maxes) > 0:
            #     delta = self.last_max - self.last_min
            #     if delta > self.hysteresis * 4:
            #         self.last_20.pop(-1)  # Remove spurious point
            #     std = np.std(self.last_20)
            #     self.hysteresis = std/2  # 1/2 standard deviation
            #     self.hysteresis = (self.hysteresis + delta / 4) / 2
            # max_twitch = self.last_max

            # a new max verified so add to list of argmax's
            if self.maxes.length() == 0 or self.max_times.signal()[-1] != self.last_tmax:
                # print('max verified at t=', self.last_tmax)
                self.max_times.append(self.last_tmax)
                self.maxes.append(self.last_max)
                self.max_signal = self.last_max  # Remember last maximum signal
            self.last_min = self.last_max
            self.last_tmin = self.last_tmax
            self.update_max = False  # Now looking for min
            direction = +1  # max detected

        elif not self.update_max and v > self.last_min + self.hysteresis:
            # Update hysteresis dynamically (but slowly)
            # if len(self.maxes) > 0:
            #     delta = self.last_max - self.last_min
            #     if delta > self.hysteresis * 4:
            #         self.last_20.pop(-1)  # Remove spurious point
            #     std = np.std(self.last_20)
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

        return direction

    def signal(self):
        return self.tracker.signal()

    def signal_offset(self):
        return self.tracker.buf_start()

    def time(self):
        return self.milliseconds.signal()

    def maximums(self) -> []:
        # value at signal maxes
        return self.maxes.signal()

    def time_of_maximums(self) -> []:
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


class SignalTracker:
    # Track time signal

    def __init__(self):
        self.tracking = []

    def append(self, v):
        self.tracking.append(v)

    def signal(self) -> []:
        return self.tracking

    def reset(self):
        self.tracking = []


class CircularSignalTracker:
    # Track time signal within circular buffer

    def __init__(self):
        self.buflen = 200
        self.tracking = CircularBuffer(self.buflen)

    def append(self, v):
        self.tracking.append(v)

    def signal(self) -> np.array:
        return self.tracking.signal()

    def reset(self):
        self.tracking.reset()


class PostAndMagnetTracker:
    # added more initialization parameters
    def __init__(self, fps, nframes, thresh_range, area_range, roi_post, roi_magnet, roi=(72, 420, 199, 270)):
        """ PostAndMagnetTracker Constructor

        tracker = PostAndMagnetTracker(thresh_range,area_range)

        :param thresh_range: is a 2 element list of (min,max) threshold ranges for magnet and post
        :param area_range: is a 2 element list of (min,max) area ranges for magnet and post
        :param roi is a 4 element tuple (xmin,xmax,ymin,ymax)
        """
        frames_to_plot = 200
        self.maxTracker = MaxTracker(frames_to_plot)
        self.fps = fps
        self.nframes = nframes
        self.thresh_range = thresh_range
        self.area_range = area_range
        self.roi_post = roi_post
        self.roi_magnet = roi_magnet
        self.extent = 0.6
        self.aspectratio = (0.5, 2.0)
        self.circularity = 0.5
        self.roi = (0,sensor.width(), 0, sensor.height())
        
        self.extra_print_string = ''

        self.wait_until = 0
        self.millisecs = frames_to_plot / fps * 1000 # Max number of milliseconds to plot

        # Determine good x-axis ticks
        self.determine_best_xticks((0, self.millisecs))

        self.font_scale = 3
        self.image_scale = 3
        self.spring_constant = 0.159

        self.automatic_thresh = False  # Automatically determine the thresholds

        # Lazily initialized when first image is processed
        self.passive_deflection = None  # deflection in pixels
        self.dist_neutral = None  # distance in pixels
        self.microns_per_pixel = None
        self.angle = None
        self.trans = None  # Rotation transform
        self.post_centroid = None
        self.twitch_deflection = 0
        # Lazily initialized title and oscilloscope areas
        self.zoom_rect = None
        self.title_rect = None
        self.oscilloscope_rect = None
        self.axes_rect = None
        self.rows = None
        self.cols = None
        
        self.stats_m = None
        # Lazily initialized title and oscilloscope areas
        self.zoom_rect_bigger = None
        self.title_rect_bigger = None
        self.axes_rect_bigger = None
        self.rows_bigger = None
        self.cols_bigger = None

        self.ylim_left  = [10000.0,-10000.0]  # Outside expected range
        self.ylim_right = [10000.0,-10000.0]  # Outside expected range

    def setFps(self, fps):
        self.fps = fps

    # This should be private
    def initPassive(self, img, stats_m, stats_p, postManualFlag, postManual):
        title_height = 80
        left_padding = 120
        right_padding = 120
        top_padding = 10
        bottom_padding = 25
        zoom_factor = 1.0

        if postManualFlag:
            centroid_p = postManual
        elif stats_m == None or stats_p == None:
            return (0,0), (0,0)
        
        # Initialize state from first image
        # Assume all images will be the same size
        height = sensor.height()
        width = sensor.width()
        print(stats_m)
        print(stats_p)
        
        self.automatic_thresh = False
        if len(stats_m) == 0:
            centroid_m = np.array((0, 0))
        else:
            centroid_m = np.array((stats_m[0].cx(), stats_m[0].cy()))
        if postManualFlag:
            centroid_p = np.array(postManual)
        else:
            if len(stats_p) == 0:
                centroid_p = np.array(postManual)
            else:
                centroid_p = np.array((stats_p[0].cx(), stats_p[0].cy()))
        if postManualFlag:
            centroid_p = np.array(postManual)
        dx = centroid_m[0] - centroid_p[0]
        dy = centroid_m[1] - centroid_p[1]

        # Determine the passive deflection
        dist_current = np.linalg.norm(centroid_m - centroid_p)

        if self.passive_deflection is not None:
            # Follow max distance up
            self.dist_neutral = max(self.dist_neutral, dist_current)
            self.passive_deflection = self.dist_neutral - dist_current  # Distance between posts during relaxation of issue
        else:
             # # Determine the amount of rotation necessary to get the post and magnet to be horizontal
            # self.angle = math.atan2(dy, dx)
            #
            # # Make rotation transform
            # self.trans = makehgtform('translate', (width // 2, height // 2, 0),
            #                          'zrotate', -self.angle,
            #                          'translate', (-width // 2, -height // 2, 0))

            # dist_current = distance.euclidean(centroid_m, centroid_p)  # distance between post and magnet
            self.dist_neutral = dist_current
            # print("dist_neutral=",self.dist_neutral)
            # print("dist_current=",dist_current)
            self.passive_deflection = self.dist_neutral - dist_current  # Distance between posts

            # Determine the scale of the image using the dist_neutral
            self.microns_per_pixel = 8000 / self.dist_neutral  # For now assume the magnet and post are 8 mm away

            # # Rotate centroid (for zoom computation)
            # centroid_m = self.trans.apply2D(centroid_m)
            # centroid_p = self.trans.apply2D(centroid_p)

            # # zoom into post+magnet (of rotated img)
            # # Make post+magnet take up 90% of the screen (zoom in)
            center = np.floor((centroid_m + centroid_p) / 2)
            # zoom_factor = 1.75 * dist_current / max(width, height) / 2
            min_row = int(max(center[1] - math.floor(height * zoom_factor / 2), 0))
            max_row = int(min(center[1] + math.floor(height * zoom_factor / 2), height))
            min_col = int(max(center[0] - math.floor(width * zoom_factor), 0))
            max_col = int(min(center[0] + math.floor(width * zoom_factor), width))
            #
            # # Position of zoom rect
            self.zoom_rect = (min_col, min_row, max_col - min_col, max_row - min_row)

            # Remember post centroid.  Comment this out to compute the post centroid each time
            # self.post_centroid = stats_p
            if postManualFlag:
                self.post_centroid = postManual
            else:
                self.post_centroid = (stats_p[0].cx(), stats_p[0].cy())

            # Determine location of title rect
            # self.title_rect = (min_col, max(min_row - title_height, 0), max_col - min_col, title_height)
            self.title_rect = (0, 0, sensor.width(), 80)

            # Determine location of oscilloscope rect
            oscilloscope_height = max(height - max_row, 120)
            self.oscilloscope_rect = (min_col, min(max_row, height-oscilloscope_height), max_col - min_col, oscilloscope_height)

            # Determine location of axes_rect
            self.axes_rect = (min_col + left_padding,
                              self.oscilloscope_rect[1] + top_padding,
                              max_col - min_col - left_padding - right_padding,
                              oscilloscope_height - top_padding - bottom_padding)

            # Selection rectangle within image (includes title and oscilloscope areas)
            # print("min_row=",min_row," title_height=",title_height)
            # print("max_row=",max_row," oscil_height=",oscilloscope_height)
            self.rows = range(min_row - title_height, max_row + oscilloscope_height)
            self.cols = range(min_col, max_col)
        return centroid_m, centroid_p # added return values

    def computeStretch(self, displacement, maximums):
        stretch_percent = displacement / self.dist_neutral * 100  # percent of the neutral displacement
        return stretch_percent

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

    def determine_best_xticks(self, xlim):
        # Determine good x-axis ticks starting with milliseconds[0] going for self.millisecs
        # Determine good x-axis ticks
        possible_deltas = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000]  # Readable deltas
        num_ticks = [int(self.millisecs / delta) for delta in possible_deltas]
        # print('num_ticks=', num_ticks)
        good_ticks = [n if n <= 5 else 0 for n in num_ticks]
        best_tick = np.argmax(good_ticks)
        best_delta = possible_deltas[best_tick]
        best_scale = 1000 if best_delta >= 1000 else 1

        start = math.ceil(xlim[0] / best_delta) * best_delta
        end = math.ceil((xlim[0] + self.millisecs) / best_delta) * best_delta
        self.xticks = [i for i in range(start, end, best_delta)]
        # self.axis_limits = (start, start + int(self.millisecs), 0, self.axes_rect[3])

        return best_scale

    def plot(self, img, rect, x, y, color, symbol, linewidth, leftright):  # left=0 right=1
        ymin = min(y)
        ymax = max(y)
        yavg = (ymax - ymin) / 2
        if math.fabs(yavg) == 0.0:  # Avoid divide by zero
            ymin = ymin - 0.5
            ymax = ymax + 0.5

        # Update ylim based on new y
        if leftright==0:
            ymin = min(self.ylim_left[0], ymin)
            ymax = max(self.ylim_left[1], ymax)
            self.ylim_left = (ymin, ymax)
        else:  # leftright==1
            ymin = min(self.ylim_right[0], ymin)
            ymax = max(self.ylim_right[1], ymax)
            self.ylim_right = (ymin, ymax)

        # Update x range of plot
        self.millisecs = max(self.millisecs, x[-1]-x[0])

        # Plot stretch in oscilloscope area
        start = x[0]
        axis_limits = (start, start + self.millisecs, ymin - yavg * 0.1, ymax + yavg * 0.1)
        tform = makehgtform('axisToRect', (axis_limits, rect))
        if len(x.shape)==1:
            x = np.array(x).reshape((1,len(x)))
            y = np.array(y).reshape((1,len(y)))
        # print("x.shape=",x.shape," y.shape=", y.shape)
        xy = np.concatenate((np.array(x), np.array(y)), axis=0)  # 2-by-n
        # print('xy=', xy)
        xypts = tform.apply2D(xy)
        xypts = xypts.T.reshape((xypts.size // 2, 2))  # Needed for polylines
        # print("xypts.shape_new", xypts.shape)
        xypts = np.array(xypts, dtype=np.int16)

        # Show lines '-' or symbol 'o'
        if symbol == '-':
            last_pt = xypts[0]
            for i in range(1, xypts.shape[0]):
                img.draw_line(last_pt[0], last_pt[1],
                              int(xypts[i][0]), int(xypts[i][1]),
                              color, thickness=linewidth)
                last_pt = xypts[i]
        else:
            last_pt = xypts[0]
            for i in range(0, xypts.shape[0]):
                if xy[0,i] >= start:
                    img.draw_line(last_pt[0], last_pt[1],
                                  int(xypts[i][0]), int(xypts[i][1]),
                                  color, thickness=linewidth)
                    last_pt = xypts[i]
                    img.draw_circle(int(xypts[i][0]), int(xypts[i][1]), 3, color, thickness=1, fill=True)

        img.draw_circle(int(xypts[-1][0]), int(xypts[-1][1]), 5, color, fill=True)

        # Determine y limit strings (display at least two digits)
        if math.fabs(ymin) < 10 < math.fabs(ymax):
            ylim = ("%.0f" % ymin, "%.0f" % ymax)
        else:
            ylim = ("%0.1f" % ymin, "%0.1f" % ymax)

        # # determine size of each label string (aids in placement)
        label_font_size = self.font_scale
        ylim_siz = (EstTextSize(ylim[0], scale=label_font_size), EstTextSize(ylim[1], scale=label_font_size))
        baseline = (0, 0)

        # Add labels on the left
        if leftright == 0:
            img.draw_string(rect[0] - ylim_siz[0][0],
                            rect[1] + rect[3] + baseline[0] - ylim_siz[0][1],
                            ylim[0],
                            color=color, scale=label_font_size, mono_space=False)
            img.draw_string(rect[0] - ylim_siz[1][0],
                            rect[1] + baseline[0] - ylim_siz[1][1] // 2,
                            ylim[1],
                            color=color, scale=label_font_size, mono_space=False)

        # Add labels on the right
        else:
            # Add frequency axis limits
            img.draw_string(rect[0] + rect[2] + 1,
                            rect[1] + rect[3] - ylim_siz[0][1],
                            ylim[0],
                            color=color, scale=label_font_size, mono_space=False)
            img.draw_string(rect[0] + rect[2] + 1,
                            rect[1] + baseline[0] - ylim_siz[1][1] // 2,
                            ylim[1],
                            color=color, scale=label_font_size, mono_space=False)

    def computeStretch(self, tracker):
        stretch_percent = np.array(tracker.signal()) / self.dist_neutral * 100
        return stretch_percent, [],[]

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
                last_max_percent = tracker.last_max_signal() / self.dist_neutral * 100
                # Compute beat frequency from signal
                if np.any(np.diff(time_of_max) == 0):
                    print('Got zero')

                # Estimate frequency of maximums
                f = 1000.0 / np.diff(time_of_max)
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

    def computeTension(self, displacement, maximums):
        # Compute tension on flexible post
        passive_tension = self.spring_constant * self.passive_deflection * self.microns_per_pixel
        twitch_tension = self.spring_constant * displacement * self.microns_per_pixel
        return passive_tension, twitch_tension

    def processImage(self, img, capture_time_ms, value, plotting_parameters, func, postManualFlag, postManual):
        old_plotting_parameters = plotting_parameters
        
        magnet_prev_pixel_val = img.get_pixel(int(old_plotting_parameters[0][0]), int(old_plotting_parameters[0][1]))
        old_values = value
        # Locate magnet and post
        width = sensor.width()
        height = sensor.height()
        #self.thresh_range = ((0,magnet_prev_pixel_val),self.thresh_range[1])
        
        # Return statistics for the magnet and post (if found)
        if self.stats_m == None:
            img.draw_rectangle(self.roi_magnet[0], self.roi_magnet[1], self.roi_magnet[2]-self.roi_magnet[0], self.roi_magnet[3]-self.roi_magnet[1],color=255, thickness=2)
            stats_m = locate_magnet(img, self.thresh_range, self.area_range, self.roi_magnet, self.aspectratio, self.extent)
            self.stats_m = stats_m
        elif len(self.stats_m) == 0:
            img.draw_rectangle(self.roi_magnet[0], self.roi_magnet[1], self.roi_magnet[2]-self.roi_magnet[0], self.roi_magnet[3]-self.roi_magnet[1],color=255, thickness=2)
            stats_m = locate_magnet(img, self.thresh_range, self.area_range, self.roi_magnet, self.aspectratio, self.extent)
            self.stats_m = stats_m
        else:
            #print(self.stats_m)
            bbox = self.stats_m[0].rect()
            #print(bbox)            
            #utime.sleep_ms(1000)

            mag_roi = (bbox[0]-100, bbox[0]+bbox[2]+100, bbox[1]-100, bbox[1]+bbox[3]+100)
            #print(mag_roi)
            s = img.get_statistics(roi=bbox)
            mag_thresh = (0,s.max())
            img.draw_rectangle(mag_roi[0],mag_roi[2],mag_roi[1]-mag_roi[0],mag_roi[3]-mag_roi[2],color=255, thickness=2)
            img.draw_rectangle(bbox[0], bbox[1],bbox[2],bbox[3], color=155, fill=True)
            #print(mag_roi)
            print("mean:{}, median:{}, mode:{}, stdev:{}, min:{}, max:{}, thresh:{}, stats:{}".format(s[0], s[1], s[2], s[3], s[4], s[5], self.thresh_range[0][1], self.stats_m))
            stats_m = locate_magnet(img, (self.thresh_range[0], self.thresh_range[1]), self.area_range, mag_roi, self.aspectratio, self.extent)
            #stats_m = locate_magnet(img, self.thresh_range, self.area_range, self.roi_magnet, self.aspectratio, self.extent)
            self.stats_m = stats_m
            self.mag_roi = mag_roi
            
            #print(stats_m)
            
        if len(stats_m) > 0:
            stats_m = [stats_m[0]]
            centroid_m = np.array((stats_m[0].cx(), stats_m[0].cy()))
            #img.draw_rectangle(stats_m[0].rect()[0],stats_m[0].rect()[1],stats_m[0].rect()[2],stats_m[0].rect()[3], color=255, thickness=1, fill=False)
        
        elif len(stats_m) == 0:
            centroid_m = np.array(old_plotting_parameters[0])
            #self.extra_print_string += "No Magnet Centroid"
        elif stats_m == None:
            centroid_m = np.array(old_plotting_parameters[0])
            #self.extra_print_string += "None Magnet Centroid"
        
        #if self.post_centroid is None:
        if postManualFlag:
            stats_p = None
            centroid_p = postManual
        else:
            stats_p = locate_post(img, self.thresh_range, self.area_range, self.roi_post, self.circularity)
            # Choose best one
            if len(stats_p) > 0:
                stats_p = [stats_p[0]]  # Choose best one
                centroid_p = np.array((stats_p[0].cx(), stats_p[0].cy()))
                
            elif len(stats_p) == 0:
                centroid_p = np.array(old_plotting_parameters[1])
                #self.extra_print_string += ", No Post Centroid"
            elif stats_p == None:
                centroid_p = np.array(old_plotting_parameters[1])
                #self.extra_print_string += ", None Post Centroid"
            
        dist_current = np.linalg.norm(centroid_m - centroid_p)
        twitch_deflection = dist_current - self.dist_neutral
        self.twitch_deflection = twitch_deflection
        # print('twitch deflection=', twitch_deflection)
        # Track the capture time, deflection and max
        self.maxTracker.process(capture_time_ms, twitch_deflection)
        
        value = func(self, self.maxTracker)  # returns 
        
        milliseconds = self.maxTracker.time()
        time_of_max = self.maxTracker.time_of_maximums()
        plotting_parameters = (centroid_m, centroid_p, milliseconds, time_of_max, value)
        #centroid_m, centroid_p, milliseconds, time_of_max, value = plotting_parameters
        #ret, annotated = self.showTrackingOscilloscope(img, centroid_m, centroid_p, self.maxTracker.time(), time_of_max, value)
        #plotting_parameters = (centroid_m, centroid_p, self.maxTracker.time(), time_of_max, value)
        if not postManualFlag:
            for stat in stats_p:
                x,y,r = stat.enclosing_circle()
                
                img.draw_circle(x,y,r,color=(0,255,0), thickness=2)
                
                x,y,w,h = stat.rect()
                img.draw_rectangle(x,y,w,h,color=(255,255,0), thickness=4)
        
        return value, plotting_parameters
            
        # First time through, set the passive deflection (in pixels)
        # Also set the zoom in ranges and the rotation transform

        # First time through or until first max (within first two seconds)
        # if self.passive_deflection is None or (len(self.maxTracker.maximums()) < 1 and capture_time_ms < self.wait_until):
        #     self.initPassive(img, stats_m, stats_p)
        #     if self.passive_deflection is None:
        #         self.wait_until = capture_time_ms + 2000

        # centroid_m = np.array((stats_m[0].cx(), stats_m[0].cy()))
        # centroid_p = np.array((stats_p[0].cx(), stats_p[0].cy()))

        # dist_current = np.linalg.norm(centroid_m - centroid_p)
        # dist_current = distance.euclidean(centroid_m, centroid_p)  # In pixels
        # twitch_deflection = dist_current - self.dist_neutral
        # print('twitch deflection=', twitch_deflection)

        # Track the capture time, deflection and max
        # self.maxTracker.process(capture_time_ms, twitch_deflection)

        # Compute the application-specific value
        # maximums = self.maxTracker
        # displacement = self.maxTracker
        # value = func(self, self.maxTracker)  # returns (stretch_percent, freq at max, last max stretch percent

        # # Early return (no plotting)
        # return 0, [], value

        # # Return with centroid tracking indicators
        # ret = self.showTracking(img, centroid_m, centroid_p)
        # return ret, img, value

        # Return with oscilloscope plotting
        milliseconds = self.maxTracker.time()
        time_of_max = self.maxTracker.time_of_maximums()
        maximums = self.maxTracker.maximums()
        centroid_m = (stats_m[0].cx(), stats_m[0].cy())
        if postManualFlag:
            centroid_p = postManual
        else:
            centroid_p = (stats_p[0].cx(), stats_p[0].cy())

        plotting_parameters = (centroid_m, centroid_p, milliseconds, time_of_max, value)
        #print(plotting_parameters)
        #ret, annotated = self.showTrackingOscilloscope(img, centroid_m, centroid_p, milliseconds, time_of_max, value)
        #outputs = (passiveLengthCalcFlag, self.dist_neutral, self.passive_deflection)
        if not postManualFlag:
            for stat in stats_p:
                x,y,r = stat.enclosing_circle()
                
                img.draw_circle(x,y,r,color=(0,255,0), thickness=2)
                
                x,y,w,h = stat.rect()
                img.draw_rectangle(x,y,w,h,color=(255,255,0), thickness=4)
        return value, plotting_parameters

        # ret, annotated = self.showTrackingOscilloscope(img, centroid_m, centroid_p,
        #                                                milliseconds, time_of_max, value)
        # return ret, annotated, value

    def showTrackingOscilloscope(self, img, centroid_m, centroid_p, milliseconds, time_of_max, value, current_well):
        # Highlight the location of the magnet and post centroids
        # Show plot of value as a function of milliseconds
        # maximums is the same length as value[1]

        width = sensor.width()
        height = sensor.height()

        # Block out title and oscilloscope areas
        img.draw_rectangle(self.title_rect, color=10, fill=True)
        # img.draw_rectangle(self.oscilloscope_rect, color=0, fill=True)

        # Indicate the axes area
        # img.draw_rectangle(self.axes_rect, color=255, thickness=2)

        # Draw centroid in image
        ret = self.showTracking(img, centroid_m, centroid_p)

        # Plot the signal like an oscilloscope = some function of twitch_deflection and max
        if len(value) > 0:
            stretch = value[0]  # From computeStretchFrequency
            beat_freq = value[1]  # From computeStretchFrequency
            # signal_offset = self.maxTracker.signal_offset()

            # Plot signal and beat freq in oscilloscope area
            # if len(beat_freq) > 0:
            #     self.plot(img, self.axes_rect, milliseconds, stretch, 127, '-', 1, 0)

            #     # Show frequency as symbols
            #     n = len(beat_freq)
            #     ms = np.zeros((n+2,),dtype=np.float)
            #     f  = np.zeros((n+2,),dtype=np.float)
            #     f[0] = beat_freq[0]
            #     ms[0] = milliseconds[0]
            #     for i in range(0,n):
            #         f[i+1] = beat_freq[i]
            #         ms[i+1] = time_of_max[i]

            #     f[n+1] = beat_freq[-1]
            #     ms[n+1] = milliseconds[-1]

            #    # print("f",f,"  ms=", ms, "milliseconds[-1]=", milliseconds[-1])

            #     self.plot(img, self.axes_rect, ms, f, 255, 'o', 1, 1)
            # else:
            #     # Plot stretch in oscilloscope area
            #     self.plot(img, self.axes_rect, milliseconds, stretch, 127, '-', 1, 0)

            if current_well == 1:
                well_row = 'A'
            elif current_well == 2:
                well_row = 'B'
            elif current_well == 3:
                well_row = 'C'
            elif current_well == 4:
                well_row = 'D'
            else:
                well_row = 'E' # defaults to something it can't be, in case there's a faulty value
            
            # Determine title
                
            title = "Row {}".format(well_row)
            title += "     {} s".format(round(milliseconds[-1]/1000,1))
            title += "\nStretch: %4.2f%%" % stretch[-1]
            #if len(beat_freq) > 0:
                #title = title + ("\nBeat freq: %.2f Hz" % beat_freq[-1])

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

    def showXTicks(self, img, xlim, color):
        # Ticks is an array of tick positions
        # xlim is (xmin,xmax) axis limits
        tick_length = 10  # In pixels
        color = 255
        linewidth = 1

        # Axis tform for tick label placement
        # Value is (axis,rect) where axis is (xmin,xmax,ymin,ymax) and rect is (left,top,width,height)
        # maps (xmin,ymax) to (left,top) and (xmax,ymin) to (left+width,top+height)
        # For this tform, the y axis is in pixels
        axis_limits = (xlim[0], xlim[1], 0, self.axes_rect[3])
        # print("tick axis_limits=", axis_limits)
        tform = makehgtform('axisToRect',(axis_limits, self.axes_rect))

        # Draw ticks and tick strings
        xscale = self.determine_best_xticks(xlim)
        for t in self.xticks:
            # print('tick at ', t)
            xy = np.array([[t, 0], [t, tick_length]])
            # print("xy=", xy, " shape=", xy.shape)
            xypts = tform.apply2D(xy.T).T
            # s = tform.apply2D(np.array([t, 0]))
            # e = tform.apply2D(np.array([t, tick_length]))
            # print("xypts=", xypts, " shape=", xypts.shape,  "\ns=", s, " e=", e)
            img.draw_line(int(xypts[0][0]), int(xypts[0][1]),
                          int(xypts[1][0]), int(xypts[1][1]),
                          color, thickness=linewidth)
            if xscale==1000:
                label = "%.1fs" % (float(t) / xscale)
            else:
                label = "%.0fms" % t
            position = tform.apply2D(np.array([t, 0]))
            # print("label shift =", len(label)*4*self.font_scale)
            img.draw_string(int(position[0]) - len(label)*4,
                            int(position[1]),
                            label,
                            scale=self.font_scale, color=127, mono_space=False)

    def showTracking(self, img, centroid_m, centroid_p):
        # Locate magnet and post
        width = sensor.width()
        height = sensor.height()

        # # Apply rotation to img and centroids
        # M = cv2.getRotationMatrix2D((width / 2, height / 2), self.angle * 180 / math.pi, 1)
        # rotated = cv2.warpAffine(img, M, (width, height))
        #        rotated = ndimage.rotate(img, self.angle * 180 / math.pi, reshape=False)
        # centroid_m = self.trans.apply2D(centroid_m)
        # centroid_p = self.trans.apply2D(centroid_p)

        # Show location of zoom rect
        # cv2.rectangle(rotated,
        #               (self.zoom_rect[0], self.zoom_rect[1]),
        #               (self.zoom_rect[0] + self.zoom_rect[2], self.zoom_rect[1] + self.zoom_rect[3]),
        #               (255, 0, 0),
        #               2)

        # # Block out title and oscilloscope areas
        # cv2.rectangle(img,
        #               (self.title_rect[0], self.title_rect[1]),
        #               (self.title_rect[0] + self.title_rect[2], self.title_rect[1] + self.title_rect[3]),
        #               (0, 0, 0),
        #               -1)
        # cv2.rectangle(img,
        #               (self.oscilloscope_rect[0], self.oscilloscope_rect[1]),
        #               (self.oscilloscope_rect[0] + self.oscilloscope_rect[2],
        #                self.oscilloscope_rect[1] + self.oscilloscope_rect[3]),
        #               (0, 0, 0),
        #               -1)
        #
        # # Indicate the axes area
        # cv2.rectangle(img,
        #               (self.axes_rect[0], self.axes_rect[1]),
        #               (self.axes_rect[0] + self.axes_rect[2], self.axes_rect[1] + self.axes_rect[3]),
        #               (255, 255, 255),
        #               1)

        # Draw centroid
        img.draw_circle(int(centroid_m[0]), int(centroid_m[1]), 5, color=200, fill=True)  # marker
        img.draw_circle(int(centroid_p[0]), int(centroid_p[1]), 10, color=127, fill=True)  # marker
        
        return 0

def nonzero(x):
    # Return positional argument of the nonzero elements
    nz = []
    for i in range(0,len(x)):
        if int(x[i]) != 0:
            nz.append(i)

    return nz


def nonzero_runs(y):
    # Returns arrays or runs with run length.  Both could be empty (length 0)

    # Augment y with points before and after
    z = np.zeros((1,))
    # print("z shape=",z.shape)
    # print("y shape=", np.array(y).shape)
    yaug = np.concatenate((z, np.array(y), z), axis=0)

    # Turn yaug into a boolean
    yaug = np.array(yaug > 0, dtype=np.int16)  # .astype(int)

    # Find the beginning index of constant runs within boolean array y and associated runlen
    dy = np.diff(yaug)
    print([i for i in dy])
    print([i for i in dy > 0])
    print([i for i in dy < 0])
    
    run_begin = nonzero(dy > 0)
    run_end = nonzero(dy < 0)
    # print("run_begin=",run_begin)
    # print("run_end=",run_end)
    # if run_begin is not None and run_end is not None:
    print(run_begin)
    print(run_end)
    runlen = np.array(run_end) - np.array(run_begin)
    # print("run_len=", runlen)
    # else:
    #     runlen = None

    return run_begin, runlen


def getCentroid(d):
    return [d.cx(), d.cy()]


def getExtent(d):
    return d.extent()


def getAspectRatio(d):
    return float(d.w()) / float(d.h())


def getBoundingBox(d):
    return d.rect()


def determineThresholds(img, area_range, roi, roi_post, roi_magnet, visualize=False):
    # determine segmentation thresholds via exhaustive search
    # returns thresh_range,area_range

    print("area_range=", area_range)
    print("======== determineThresholds =========")
    print("visualize = ", visualize)
    # Within reasonable range search for magnet and post
    r = range(0, 150)
    data = []
    for i in r:
        thresholds = ((0, i), (0, i))
        # Get all blobs
        if visualize:
            print("i = %d" % i, end=': ')
        stats_m = locate_magnet(img, thresholds, area_range, roi_magnet, aspectratio = (0.5, 2.0), extent=0.5)
        stats_p = locate_post(img, thresholds, area_range, roi_post, circularity = 0.5)
        # stats_m, stats_p = locate_magnet_and_post(img, thresholds, area_range, roi)
        data.append((i, len(stats_m), len(stats_p)))  # Remember number of blobs detected
        if visualize:
            VisualizeThresholdSearch(img, i, stats_m, stats_p)
        print(stats_m)

    # Extract data
    idx = [x[0] for x in data]
    nblobs_m = [x[1] for x in data]  # num blobs for magnet thresh
    nblobs_p = [x[2] for x in data]  # num blobs for post thresh
    # centroid_m = [getCentroid(x[1]) for x in data]

    # Determine runs for magnet
    # Choose run with maximum length and then set the threshold in the middle of the run
    runs_m, runlen_m = nonzero_runs(nblobs_m)
    if len(runlen_m) > 0:
        runs_m = np.array([runs_m], dtype=np.uint16)
        # print("runs_m=",runs_m)
        # print("runs_m.shape=",runs_m.shape)
        # print("runlen_m=", runlen_m)
        runmax_m = np.argmax(np.array([runlen_m]))
        # print("runmax_m=",runmax_m)
        idx_m = runs_m[0][runmax_m]
        # print("idx_m=",idx_m)
        # print("runlen_m[0]=",runlen_m[0])
        thresh_m = [0, int(r[idx_m] + runlen_m[runmax_m] // 2)]
        # print("thresh_m=", thresh_m)
    else:
        thresh_m = (0,70)
        idx_m = None

    # Determine runs for post
    # Choose run with maximum length and then set the threshold in the middle of the run
    runs_p, runlen_p = nonzero_runs(nblobs_p)
    if len(runlen_p) > 0:
        runs_p = np.array([runs_p], dtype=np.uint16)
        # print("runs_p=",runs_p)
        # print("runs_p.shape=",runs_p.shape)
        runmax_p = np.argmax(np.array([runlen_p]))
        # print("runmax_p=",runmax_p)
        idx_p = runs_p[0][runmax_p]
        thresh_p = (0, int(r[idx_p] + runlen_p[runmax_p] // 2))
    else:
        thresh_p = (0,10)
        idx_p = None

    if visualize:
        print("=========== Summary threshold stats ===========")
        print("thresh_m=", thresh_m)
        print("thresh_p=", thresh_p)
        print("nblobs_m=", nonzero(nblobs_m) )
        print("nblobs_p=", nonzero(nblobs_p) )
        print("===============================================")
    else:
        print("thresh=", thresh_m, thresh_p)

        #
        # Show plot of num blobs as a function of threshold
        # fig, ax = mp.subplots(2, 1)
        # ax[0].plot(idx, nblobs_m)
        # # ax[0].plot(area[:,0],area[:,1])
        # if thresh_m is not None:
        #     ax[0].plot(thresh_m[1], nblobs_m[idx_m], 'o')  # Chosen thresh
        # ax[0].set_ylabel('# magnet-like blobs')
        # ax[1].plot(idx, nblobs_p)
        # # ax[1].plot(area[:,0],area[:,2])
        # if thresh_p is not None:
        #     ax[1].plot(thresh_p[1], nblobs_p[idx_p], 'o')  # Chosen thresh
        # ax[1].set_ylabel('# post-like blobs')
        # mp.show()
        # end of show plot

    return (thresh_m, thresh_p), area_range


def thresholdFromVideo(vfile, visualize=False):
    # Determine appropriate threshold from the first frame of video
    thresh_range = ((0, 120), (0, 60))
    area_range=((300, 800), (1500, 3000))
    roi = (128, 553, 179, 366)

    # Open video file
    cap = cv2.VideoCapture(vfile)
    nframes = cap.get(cv2.CAP_PROP_FRAME_COUNT)

    if nframes >= 1:
        ret, frame = cap.read()
        if ret:
            thresh_range, area_range = determineThresholds(frame, area_range, roi, visualize=visualize)

    cap.release()

    return thresh_range, area_range


def blob_properties(contours, min_area):
    """ Compute blob statistics from contours
        Produces a list of stats dictionaries (one for each contour)
    """
    stats = []
    i = 0
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue  # Go to next c

        # perimeter = contours[0].size()
        perimeter = cv2.arcLength(c, True)

        # Compute centroid via moments
        M = cv2.moments(c)
        if M["m00"] != 0:
            cX = M["m10"] / M["m00"]
            cY = M["m01"] / M["m00"]
        else:
            cX, cY = 0, 0

        # Statistics
        x1, y1, w, h = cv2.boundingRect(c)
        aspect_ratio = h / float(w)
        rect_area = w * h
        extent = float(area) / rect_area

        if perimeter > 0:
            circularity = 4 * math.pi * float(area) / (float(perimeter) * perimeter)
        else:
            circularity = 0

        # (xa, ya), (MA, ma), angle = cv2.fitEllipse(cnt)
        # rect = cv2.minAreaRect(cnt)
        # (xc, yc), radius = cv2.minEnclosingCircle(cnt)
        # ellipse = cv2.fitEllipse(cnt)
        # rows, cols = img.shape[:2]

        # Add parameters to list
        props = {"id": i + 1,
                 "Area": area,
                 "Centroid": (cX, cY),
                 "Perimeter": perimeter,
                 "BoundingBox": (x1, y1, w, h),
                 "AspectRatio": aspect_ratio,
                 "RectArea": rect_area,
                 "Extent": extent,
                 "Circularity": circularity
                 }
        stats.append(props)
        i += 1

    return stats


def FilterByArea(stats, area_range) -> []:
    good = []
    for props in stats:
        area = props.area()
        if area_range[0] <= area <= area_range[1]:
            good.append(props)

    return good


def FilterByBlobColor(stats, bw) -> []:
    good = []
    for props in stats:
        cx = props.cx()
        cy = props.cy()
        v = bw[cy, cx]
        if v != 0:
            good.append(props)

    return good


def FilterByExtent(stats, e) -> []:
    good = []
    for props in stats:
        extent = props.extent()
        if extent >= e:
            good.append(props)

    return good


def FilterByAspectRatio(stats, ar) -> []:
    good = []
    for props in stats:
        
        aspect_ratio = getAspectRatio(props)
        # modified to optionally accept upper and lower bounds of acceptable aspect ratios
        if len(ar) == 1:
            if aspect_ratio >= ar:
                good.append(props)
        elif len(ar) == 2:
            if aspect_ratio >= ar[0] and aspect_ratio <= ar[1]:
                good.append(props)

    return good


def FilterByCircularity(stats, circ, ar_range=(0.7, 1.3)) -> []:
    good = []
    for props in stats:
        # Expect circular blobs to be bounded by a "square" and high circularity
        circularity = props.roundness()
        aspect_ratio = props.h() / props.w()
        if circularity >= circ and ar_range[0] < aspect_ratio < ar_range[1]:
            good.append(props)

    return good


def FilterByROI(stats, roi_x, roi_y) -> []:
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

def GetAreas(stats):
    areas = []
    for blob in stats:
        areas.append(blob.area())
    return np.array(areas)

def OrderByExtent(stats) -> [{}]:
    """ Return sorted array of statistics dictionaries sorted by Extent property
    """
    sorted_stats = sorted(stats, key=ExtentKey, reverse=True)
    return sorted_stats


def OrderByCircularity(stats) -> [{}]:
    """ Return sorted array of statistics dictionaries sorted by Circularity property
    """
    sorted_stats = sorted(stats, key=CircularityKey, reverse=True)
    return sorted_stats


def ShowCentroidsOnImage(img, stats):
    for i in range(0, len(stats)):
        centroid = np.array([stats[i].cx(),stats[i].cy()]).astype(int)
        img.putText(img, "%d" % i, centroid, cv2.FONT_HERSHEY_SIMPLEX, 0.25, (0, 0, 255), 1)
        # cv2.circle(img, centroid, 3, (0,0,255), 2)
    cv2.imshow('Centroids on image', img)


def ShowContours(contours, hierarchy, m, n):
    # Useful for debugging the blob detection
    dst = np.zeros((m, n, 3), dtype='uint8')
    cv2.drawContours(dst, contours, -1, (0, 255, 0), 1)
    cv2.imshow("Contours", dst)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def EstTextSize(str, scale=1.0):
    # Assume each letter fits inside 8x10 pixel box at scale=1
    n = len(str)
    return (int(n*scale*8), int(scale*10))  # (width, height) in pixels

def ShowLabelImage(bw):
    # show label image with the full range of colors
    (numcomp, labels, stats, centroids) = cv2.connectedComponentsWithStats(bw)
    lmat = np.zeros(labels.shape, dtype=np.uint8)
    for i in range(0, numcomp):
        lmat[labels == i] = i * 256 // numcomp
    lmat = cv2.applyColorMap(lmat, cv2.COLORMAP_JET)
    cv2.imshow('lmat', lmat)


def VisualizeThresholdSearch(img, thresh, stats_m, stats_p):
    # Show image with blobs detected as movie
        # Assume the image is grayscale
        #img_copy = img.copy(copy_to_fb=True) # These lines place the
        img = img.binary([(0,thresh)])   # bw into the frame buffer
        img = img.to_rainbow()
        
        
        for blob in stats_m:
            img.draw_circle(blob.cx(), blob.cy(), 5, color=30, fill=True)  # marker
            if blob.elongation() < 0.5:
                img.draw_edges(blob.min_corners(), color=30)
                img.draw_line(blob.major_axis_line(), color=30)
                img.draw_line(blob.minor_axis_line(), color=30)
            # These values are stable all the time.
            img.draw_rectangle(blob.rect(), color=30)
            img.draw_cross(blob.cx(), blob.cy(), color=30)
            # Note - the blob rotation is unique to 0-180 only.
            # img.draw_keypoints([(blob.cx(), blob.cy(), int(blob.rotation_deg()))], size=40, color=127)

        for blob in stats_p:
            img.draw_circle(blob.cx(), blob.cy(), 5, color=10, fill=True)  # marker
            if blob.elongation() < 0.5:
                img.draw_edges(blob.min_corners(), color=10)
                img.draw_line(blob.major_axis_line(), color=10)
                img.draw_line(blob.minor_axis_line(), color=10)
            # These values are stable all the time.
            # img.draw_rectangle(blob.rect(), color=127)
            img.draw_cross(blob.cx(), blob.cy(), color=10)
            circ = blob.enclosing_circle()
            img.draw_circle(circ[0], circ[1], circ[2], color=10)
            # Note - the blob rotation is unique to 0-180 only.
            # img.draw_keypoints([(blob.cx(), blob.cy(), int(blob.rotation_deg()))], size=40, color=127)

        img.draw_string(20, 20, "%d" % thresh, scale=3, color=100, mono_space=False)  # Show the threshold
        utime.sleep_ms(100)

def locate_magnet(img, thresh_range, area_range, roi=(0,0,sensor.width(), sensor.height()), aspectratio = (0.9,10), extent = 0.6) -> ({}, {}):
    height = sensor.height()
    width = sensor.width()

    # test = np.array([1, 2, 3]) * width
    # print(test)
    # Region of interest in normalized image coordinates(vertical center strip)
    # roi_x = np.array([0.45,.57]) * (width - 1) + 1
    # roi_y = np.array([0,1]) * (height - 1) + 1
    # roi_x = np.array([0.25, .75]) * (width - 1) + 1
    # roi_y = np.array([0, 1]) * (height - 1) + 1
    roi_x = (roi[0], roi[1])
    roi_y = (roi[2], roi[3])

    #
    # magnet detector (assume sensor is already grayscale)
    #
    # print("thresh_range=", thresh_range)
    stats_m = img.find_blobs( [thresh_range[0]], pixels_threshold=area_range[0][0], roi=(roi[0], roi[2], roi[1]-roi[0], roi[3]-roi[2]))
    
    #print(stats_m)
    # areas = GetAreas(stats_m)
    # if len(areas) > 0:
    #     print("unfiltered magnet areas from %.0f to %.0f" % (min(areas), max(areas)))

    # Filter results by area
    stats_m = FilterByArea(stats_m, area_range[0])
    # areas = GetAreas(stats_m)
    # if len(areas) > 0:
    #     print("  filtered magnet areas from %.0f to %.0f (n=%d)" % (min(areas), max(areas), len(areas)))

    # Filter by ROI
    # stats_m = FilterByROI(stats_m, roi_x, roi_y)

    # Filter by aspect ratio (expect tall approx 5:2 filter those below 0.9)
    stats_m = FilterByAspectRatio(stats_m, aspectratio)

    # Filter by extent (filter those below 0.6)
    stats_m = FilterByExtent(stats_m, extent)

    # Order by Extent
    stats_m = OrderByExtent(stats_m)

    return stats_m

def locate_post(img, thresh_range, area_range, roi, circularity=0.5) -> ({}, {}):
    height = sensor.height()
    width = sensor.width()

    # test = np.array([1, 2, 3]) * width
    # print(test)
    # Region of interest in normalized image coordinates(vertical center strip)
    # roi_x = np.array([0.45,.57]) * (width - 1) + 1
    # roi_y = np.array([0,1]) * (height - 1) + 1
    # roi_x = np.array([0.25, .75]) * (width - 1) + 1
    # roi_y = np.array([0, 1]) * (height - 1) + 1
    roi_x = (roi[0], roi[1])
    roi_y = (roi[2], roi[3])

    #
    # post detector
    #
    stats_p = img.find_blobs( [thresh_range[1]], pixels_threshold=area_range[1][0], roi=(roi[0], roi[2], roi[1]-roi[0], roi[3]-roi[2]), x_stride=5, y_stride=5)
    got_one = len(stats_p) > 0
    # print("--- stats_p ---")
    # print(stats_p)
    # areas = GetAreas(stats_p)
    # if len(areas) > 0:
    #     print("unfiltered post areas from %.0f to %.0f" % (min(areas), max(areas)))

    # # Filter by blobColor (expecting white)
    # stats_p = FilterByBlobColor(stats_p, bw)

    # Filter results by area
    stats_p = FilterByArea(stats_p, area_range[1])
    # if got_one and len(stats_p) == 0:
    #     print("  post filtered out by area")
    #     return stats_p

    # areas = GetAreas(stats_p)
    # if got_one and len(areas) > 0:
    #     print("  post areas from %.0f to %.0f (n=%d)" % (min(areas), max(areas), len(areas)))

    # Filter by ROI
    # stats_p = FilterByROI(stats_p, roi_x, roi_y)
    # if got_one and len(stats_p) == 0:
    #     print("  post filtered out by ROI ", roi)
    #     return stats_p

    # Filter by circularity
    stats_p = FilterByCircularity(stats_p, circularity)
    # if got_one and len(stats_p) == 0:
    #     print(" post filtered out by circularity")
    #     return stats_p

    # Order by Circularity
    stats_p = OrderByCircularity(stats_p)

    return stats_p
