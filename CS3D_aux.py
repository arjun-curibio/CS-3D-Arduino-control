from serial import Serial
import pyopenmv
from serial.tools.list_ports import comports
import tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.text as mtext
import matplotlib.transforms as mtransforms
from tkinter import font

class Protocol:
    def __init__(self, root):
        self.steps       = tk.IntVar(root, 300)
        self.pre_delay   = tk.IntVar(root, 1000)
        self.rise        = tk.IntVar(root, 500)
        self.hold        = tk.IntVar(root, 100)
        self.fall        = tk.IntVar(root, 500)
        self.post_delay  = tk.IntVar(root, 1000)
        self.n_cycles    = tk.IntVar(root, 1)
        self.trig        = tk.BooleanVar(root, True)
        self.assigned_to = [False, False, False, False]
    
    def update(self, steps=None, pre_delay = None, rise = None, hold = None, fall = None, post_delay = None, n_cycles = None, trig = None, assigned_to = None):
        if steps            != None: self.steps         .set(steps)
        if pre_delay        != None: self.pre_delay     .set(pre_delay)
        if rise             != None: self.rise          .set(rise)
        if hold             != None: self.hold          .set(hold)
        if fall             != None: self.fall          .set(fall)
        if post_delay       != None: self.post_delay    .set(post_delay)
        if n_cycles         != None: self.n_cycles      .set(n_cycles)
        if trig             != None: self.trig          .set(trig)
        if assigned_to      != None: self.assigned_to = assigned_to
 
class Motor:
    def __init__(self, ID):
        self.ID = ID
        self.t = 0
        self.p = 0
        self.d = 0
        self.f = 1

        self.speed_rise = 1
        self.speed_fall = 1


        self.enabled = True
        self.homed = False
        self.initialized = False
        self.homing_stage = -1

        self.cycle_count = 0
        self.n_cycles = -1

        self.assigned_protocol = True
        self.ts = [0,0,0,0,0,0]
    
    def update_motor(self,t=0, p=0, d=0, speed_rise=1, speed_fall=1, enabled=False, homed=False, initialized=False, homing_stage=-1, cycle_count= 0, n_cycles=1, ts = [0,0,0,0,0,0]):
        self.t = t
        self.p = p
        self.d = d

        self.enabled = enabled
        self.homed = homed
        self.initialized = initialized
        self.homing_stage = homing_stage

        self.cycle_count = cycle_count
        self.n_cycles = n_cycles

        self.speed_rise = speed_rise
        self.speed_fall = speed_fall

        self.ts = ts
        
    
    def display(self):
        if self.n_cycles == -1:
            temp = 'inf'
            len_temp = 5
        else:
            temp = str(self.n_cycles)
            len_temp = len(str(self.n_cycles))

        if self.enabled:
            temp2 = " ENABLED"
        else:
            temp2 = "DISABLED"

        printfunc("ID {}: t={:>5} ms, p={:>{}}/{:>{}}, speeds = [{:>{}} | {:>{}}] steps/s, {}, cycle = {:>{}}/{:<{}}, ts = {}".format(self.ID, self.t, 
            self.p, len(str(self.d)), 
            self.d, len(str(self.d)),
            round(self.speed_rise,2), len(str(round(self.speed_rise,2))),
            round(self.speed_fall,2), len(str(round(self.speed_fall,2))),
            temp2, 
            self.cycle_count, len_temp,
            temp, len_temp,
            self.ts,
            ), end=' ')



def printfunc(string, **kwargs):
    print(string, **kwargs)



# ESTABLISH CONNECTIONS
class EstablishConnections:
    def __init__(self):
        self.connected_to_motors = False
        self.connected_to_camera=False
        self.conn = None

        self.ports = comports()
        [printfunc("{}: {} ({})".format(port, desc, hwid)) for port, desc, hwid in self.ports]
        
    
    def connect_to_camera(self):
        for port, desc, hwid in comports():
            if 'OpenMV' in port or 'OpenMV' in desc or 'OpenMV' in hwid:
                try:
                    print('initing')
                    pyopenmv.init(port, baudrate=921600, timeout=0.010)
                    pyopenmv.set_timeout(1*2) # SD Cards can cause big hicups.
                    pyopenmv.stop_script() # stop any script thats working
                    pyopenmv.enable_fb(True) # enable frame buffer (NEEDS TO BE ENABLED TO GRAB FROM FRAME BUFFER)

                    try:
                        with open('D:/main.py', 'r') as f:
                            script=f.read()
                    except:
                        print('errored')
                        script = ""
                    
                    print(len(script))
                    pyopenmv.exec_script(script) # Execute the script that's located in D:/main.py
                    self.connected_to_camera = True
                    printfunc("Connected to camera.")
                except:
                    self.connected_to_camera=False
                    printfunc("Failed to connect to camera.")
                continue
            else:  
                continue
    def connect_to_motors(self):
        for port, desc, hwid in comports():
            print("{}, {}, {}".format(port, desc, hwid))
        for port, desc, hwid in comports():
            if 'OpenMV' in port or 'OpenMV' in desc or 'OpenMV' in hwid:
                continue
            elif 'Bluetooth' in port or 'Bluetooth' in desc or 'Bluetooth' in hwid:
                continue
            elif 'STMicroelectronics' in port or 'STMicroelectronics' in desc or 'STMicroelectronics' in hwid:
                continue
            else:  
                try:
                    self.conn = Serial(port, baudrate=2000000, timeout=0.01)
                    self.connected_to_motors = True
                    printfunc('Connected to motors. ')
                    continue
                except:
                    self.conn = None
                    self.connected_to_motors = False
                    printfunc('Tried to connect but failed.  Trying another or exiting.')
                    continue
        return self.conn
    
    def close(self):
        self.conn.close()



class RotationAwareAnnotation(mtext.Annotation):
    def __init__(self, s, xy, p, pa=None, ax=None, **kwargs):
        self.ax = ax
        self.p = p
        if not pa:
            self.pa = xy
        self.calc_angle_data()
        kwargs.update(rotation_mode=kwargs.get("rotation_mode", "anchor"))
        mtext.Annotation.__init__(self, s, xy, **kwargs)
        self.set_transform(mtransforms.IdentityTransform())
        if 'clip_on' in kwargs:
            self.set_clip_path(self.ax.patch)
        self.ax._add_text(self)

    def calc_angle_data(self):
        ang = np.arctan2(self.p[1]-self.pa[1], self.p[0]-self.pa[0])
        self.angle_data = np.rad2deg(ang)

    def _get_rotation(self):
        return self.ax.transData.transform_angles(np.array((self.angle_data,)), 
                            np.array([self.pa[0], self.pa[1]]).reshape((1, 2)))[0]

    def _set_rotation(self, rotation):
        pass

    _rotation = property(_get_rotation, _set_rotation)


def configure_button(button, text=None, command=None, fontsize=None, location=None, **kwargs):
    configuration = {'borderwidth':2, 'relief':'groove'}
    if text != None:
        configuration['text'] = text
    if command != None:
        configuration['command'] = command
    if fontsize != None:
        configuration['font'] = font.Font(size=fontsize)
    for key, value in kwargs.items():
        configuration[key] = value
    button.configure(configuration)
    if location != None:
        x, y, width, height = tuple(location)
        location_config = {}
        if x != -1:      location_config['x'] = x
        if y != -1:      location_config['y'] = y
        if width != -1:  location_config['width'] = width
        if height != -1: location_config['height'] = height
        button.place(location_config)
    return button

def configure_entry(entry, textvariable, location=None, fontsize=None, bind_function = None, **kwargs):
    configuration = {'textvariable':textvariable, 'background':'white', 'relief':'groove', 'borderwidth':2}
    for key, value in kwargs.items():
        configuration[key] = value
    if fontsize != None:
        configuration['font'] = font.Font(size=fontsize)
    entry.configure(configuration)
    if location != None:
        x, y, width, height = tuple(location)
        location_config = {}
        if x != -1:      location_config['x'] = x
        if y != -1:      location_config['y'] = y
        if width != -1:  location_config['width'] = width
        if height != -1: location_config['height'] = height
        entry.place(location_config)

    if bind_function != None:
        entry.bind(bind_function[0], bind_function[1])
    return entry

def configure_label(label, text, fontsize=None, location=None, anchor=None, **kwargs):
    configuration = {'background':'white', 'text':text}
    if fontsize!=None:
        configuration['font'] = font.Font(size=fontsize)
    if anchor!=None:
        configuration['anchor'] = anchor
        # label.configure(anchor='e')
    for key, value in kwargs.items():
        configuration[key] = value
    label.configure(configuration)
    if location!=None:
        x, y, width, height = tuple(location)
        location_config = {}
        if x != -1:      location_config['x'] = x
        if y != -1:      location_config['y'] = y
        if width != -1:  location_config['width'] = width
        if height != -1: location_config['height'] = height
        
        label.place(location_config)
    return label
