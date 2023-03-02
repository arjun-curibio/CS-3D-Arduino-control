from serial import Serial
import tkinter as tk
from tkinter import font
import threading
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import numpy as np
import json, sys, pyopenmv, pygame
from math import floor
from datetime import datetime
from serial.tools.list_ports import comports
from time import time
class Motor:
    def __init__(self, ID):
        self.ID = ID
        self.t = 0
        self.p = 0
        self.d = 0
        self.f = 1


        self.enabled = False
        self.homed = False
        self.initialized = False
        self.homing_stage = -1
    
    def update_motor(self,t=0, p=0, d=0, f=1, enabled=False, homed=False, initialized=False, homing_stage=-1):
        self.t = t
        self.p = p
        self.d = d
        self.f = f


        self.enabled = enabled
        self.homed = homed
        self.initialized = initialized
        self.homing_stage = homing_stage
    
    def display(self):
        printfunc('ID {}: t={}, p={}, d={}, f={}, enabled={}, homed={}, initialized={}, homing_stage={}'.format(self.ID, self.t, self.p, self.d, self.f, self.enabled, self.homed, self.initialized, self.homing_stage))


class CS3D:
    def __init__(self):
        self.root = tk.Tk()
        self.root.geometry("500x525")
        self.root.configure({"background":'white'})
        self.root.resizable(0,0)
        self.root.protocol("WM_DELETE_WINDOW", self.destroy)
        
        self.waveform_root = tk.Toplevel(master=self.root)
        self.waveform_root.configure(background='white')
        self.waveform_root.geometry("300x250")
        self.waveform_root.withdraw()
        self.entry_rise     = tk.Entry(self.waveform_root)
        self.entry_hold     = tk.Entry(self.waveform_root)
        self.entry_fall     = tk.Entry(self.waveform_root)
        self.entry_freq     = tk.Entry(self.waveform_root)
        self.entry_stretch  = tk.Entry(self.waveform_root)
        self.title_waveform_dialog = tk.Label(self.waveform_root)
        
        fig = Figure()
        self.plot_axis = fig.add_subplot(111)
        self.plot_canvas = FigureCanvasTkAgg(fig, master=self.waveform_root)
        self.plot_canvas.get_tk_widget().place(x=150, y=0, width=150, height=200)
        fig.tight_layout()

        self.ports = EstablishConnections()
        self.ports.connect_to_camera()
        self.conn = self.ports.connect_to_motors()

        pygame.init()
        self.screen = pygame.display.set_mode((640,480), pygame.DOUBLEBUF)
        self.got_image = False
        self.t0 = time()
        self.screen_font = pygame.font.SysFont("monospace", 33)
        self.bottom_screen_font = pygame.font.SysFont("monospace", 14)
        self.t_display = time()

        self.well_labels        = ['D',                         'C',                        'B',                        'A']
        self.well_locations     = [0,                           4000,                       7900,                       11500]
        self.motors             = [Motor(0),                    Motor(1),                   Motor(2),                   Motor(3)]
        self.frames             = [tk.Frame(self.root),         tk.Frame(self.root),        tk.Frame(self.root),        tk.Frame(self.root)]
        self.button_move_to     = [tk.Button(self.frames[0]),   tk.Button(self.frames[1]),  tk.Button(self.frames[2]),  tk.Button(self.frames[3])]
        self.button_retract     = [tk.Button(self.frames[0]),   tk.Button(self.frames[1]),  tk.Button(self.frames[2]),  tk.Button(self.frames[3])]
        self.button_start       = [tk.Button(self.frames[0]),   tk.Button(self.frames[1]),  tk.Button(self.frames[2]),  tk.Button(self.frames[3])]
        self.button_waveform    = [tk.Button(self.frames[0]),   tk.Button(self.frames[1]),  tk.Button(self.frames[2]),  tk.Button(self.frames[3])]
        self.button_camera_up   = [tk.Button(self.frames[0]),   tk.Button(self.frames[1]),  tk.Button(self.frames[2]),  tk.Button(self.frames[3])]
        self.button_camera_down = [tk.Button(self.frames[0]),   tk.Button(self.frames[1]),  tk.Button(self.frames[2]),  tk.Button(self.frames[3])]
        

        
        self.motor_display   = [tk.Label(self.frames[0]),    tk.Label(self.frames[1]),   tk.Label(self.frames[2]),   tk.Label(self.frames[3])]

        self.rise            = [tk.IntVar(self.waveform_root, 25),    tk.IntVar(self.waveform_root, 25),   tk.IntVar(self.waveform_root, 25),   tk.IntVar(self.waveform_root, 25)]
        self.hold            = [tk.IntVar(self.waveform_root, 50),    tk.IntVar(self.waveform_root, 50),   tk.IntVar(self.waveform_root, 50),   tk.IntVar(self.waveform_root, 50)]
        self.fall            = [tk.IntVar(self.waveform_root, 75),    tk.IntVar(self.waveform_root, 75),   tk.IntVar(self.waveform_root, 75),   tk.IntVar(self.waveform_root, 75)]
        self.freq            = [tk.DoubleVar(self.waveform_root, 1),  tk.DoubleVar(self.waveform_root, 1), tk.DoubleVar(self.waveform_root, 1), tk.DoubleVar(self.waveform_root, 1)]
        self.stretch         = [tk.IntVar(self.waveform_root, 5),     tk.IntVar(self.waveform_root, 5),    tk.IntVar(self.waveform_root, 5),    tk.IntVar(self.waveform_root, 5)]
        
        self.actual_rise     = [25, 25, 25, 25]
        self.actual_hold     = [50, 50, 50, 50]
        self.actual_fall     = [75, 75, 75, 75]
        self.actual_freq     = [5,  5,  5,  5]
        self.actual_stretch  = [5,  5,  5,  5]
        
        self.well_motor     = 0
        self.waveform_motor = 0

        self.command        = tk.StringVar()
        self.feedback_active_flag = [False, False, False, False]

        self.tracking_stretch = 0
        self.tracking_well = 0
        self.tracking_motor_homed = False
        self.tracking_recieved_motor_init_handshake=False
        self.tracking_found_max = False
        self.tracking_maxes = []
        
        self.camera_row = 'D'
        self.camera_enable = False
        self.camera_position = 0
        self.camera_well = 0
        
        self.got_stretch   = False
        self.algorithm_magnet_flag = False
        # self.waveform_root.protocol("MW_DELETE_WINDOW", self.waveform_window_close)

        # CONFIGURE WAVEFORM DIALOG
        self.configure_entry(self.entry_rise,       self.rise[0],       [10,40,30,25], 10)
        self.configure_entry(self.entry_hold,       self.hold[0],       [10,70,30,25], 10)
        self.configure_entry(self.entry_fall,       self.fall[0],       [10,100,30,25], 10)
        self.configure_entry(self.entry_freq,       self.freq[0],       [10,130,30,25], 10)
        self.configure_entry(self.entry_stretch,    self.stretch[0],    [10,160,30,25], 10)
        self.title_waveform_dialog = self.configure_label(self.title_waveform_dialog, text='Row {}'.format(self.well_labels[self.waveform_motor]), fontsize=12, justify=tk.CENTER, location=[0,0,150,35])
        
        self.configure_label(tk.Label(self.waveform_root), ": Rise time (t/T)", location=[40,40,-1,-1], fontsize=8, justify=tk.LEFT)
        self.configure_label(tk.Label(self.waveform_root), ": Hold time (t/T)", location=[40,70,-1,-1], fontsize=8, justify=tk.LEFT)
        self.configure_label(tk.Label(self.waveform_root), ": Fall time (t/T)", location=[40,100,-1,-1], fontsize=8, justify=tk.LEFT)
        self.configure_label(tk.Label(self.waveform_root), ": Frequency (Hz) ", location=[40,130,-1,-1], fontsize=8, justify=tk.LEFT)
        self.configure_label(tk.Label(self.waveform_root), ": Stretch (%)    ", location=[40,160,-1,-1], fontsize=8, justify=tk.LEFT)
        self.button_send_waveform = self.configure_button(tk.Button(self.waveform_root), "Program Row A", command=self.send_waveform, location=[0, 205, -1, 25], fontsize=12)
        self.button_check_waveform = self.configure_button(tk.Button(self.waveform_root), "Check Parameters", command=self.check_waveform, location=[0, 175, -1, 25], fontsize=12)
        self.button_send_waveform_to_all = self.configure_button(tk.Button(self.waveform_root), "Send to All", command=self.send_waveform_to_all, location=[130, 205, 130, 25], fontsize=12)
        
        # CONFIGURE GLOBAL COMMAND BUTTONS
        self.button_reset_camera_position   = self.configure_button(tk.Button(self.root), "Reset Camera\nPosition", self.reset_camera_position, 10, [100, 430, 100, 50])
        self.button_retract_all_motors      = self.configure_button(tk.Button(self.root), "Retract All\nMotors",    self.retract_all_motors,    10, [200, 430, 100, 50])
        self.button_stop_all_motors         = self.configure_button(tk.Button(self.root), "Stop All\nMotors",       self.stop_all_motors,       10, [0, 430, 100, 50])
        
        self.frame_initialization           = tk.Frame(self.root)
        self.button_initialize_camera       = self.configure_button(tk.Button(self.frame_initialization), "Initialize\nCamera",     self.initialize_camera,     10, [20, 0, 100, 40])
        self.button_deinitialize_camera     = self.configure_button(tk.Button(self.frame_initialization), "De-initialize\nCamera",  self.deinitialize_camera,   10, [20, 40, 100, 40])
        self.button_home_motor              = self.configure_button(tk.Button(self.frame_initialization), "Home Motor",             self.home_motor,            10, [20, 80, 100, 25])
        self.button_adj_up                  = self.configure_button(tk.Button(self.frame_initialization), "-",                      lambda: self.send_adj(0),   10, [0, 82.5, 20, 20])
        self.button_adj_down                = self.configure_button(tk.Button(self.frame_initialization), "+",                      lambda: self.send_adj(1),   10, [120, 82.5, 20, 20])
        self.button_toggle_feedback         = self.configure_button(tk.Button(self.frame_initialization), "Toggle Feedback\n(off)", command=self.toggle_feedback, fontsize=10, location=[10, 110, 120, 40])
        
        self.entry_command                  = self.configure_entry(tk.Entry(self.root),     textvariable=self.command, location=[25, 500, -1, -1], fontsize=10 )
        self.entry_command.bind('<Return>', func=lambda val: self.send_command())
        
        printfunc('adding frames for each camera')
        self.frame_initialization.configure({  'background':'white', 
                                        'borderwidth':0,
                                        'relief':'groove', 
                                        'width':150, 
                                        'height':150})
        self.frame_initialization.place(x=330, y=315)

        for i in range(4):
            def move_camera(motor=i):
                string = "C{},{}\n".format(motor, self.well_locations[motor])
                self.write_to_motors(string)
                self.button_move_to[self.well_motor].configure(text='Move Camera\nto Row {}'.format(self.well_labels[self.well_motor]))
                self.button_move_to[self.well_motor].configure(borderwidth=2)
                self.button_move_to[self.well_motor].configure(state='normal')
                self.well_motor = motor
                self.button_move_to[self.well_motor].configure(text='Camera Under\nRow {}'.format(self.well_labels[self.well_motor]))
                self.button_move_to[self.well_motor].configure(borderwidth=7)
                self.button_move_to[self.well_motor].configure(state='disabled')
                
                self.frame_initialization.place(x=330, y=315-(motor*105))
                # self.configure_button(self.button_initialize_camera,    location=[350, 315-(motor*105), 100, 40])
                # self.configure_button(self.button_home_motor,           location=[350, 350-(motor*105), 100, 25])
                # self.configure_button(self.button_adj_down,             location=[450, 350-(motor*105), 20, 20])
                # self.configure_button(self.button_adj_up,               location=[330, 350-(motor*105), 20, 20])
                # self.configure_button(self.button_toggle_feedback,      location=[340, 400-(motor*105), 120, 40])
                # print([[350, 315-(i*105), 100, 40]])
            def retract_motor(motor=i):
                string = "X{}\n".format(motor)
                self.write_to_motors(string)
            def enable_motor(motor=i):
                string = "M{}\n".format(motor)
                self.write_to_motors(string)
                if self.motors[motor].enabled:  self.configure_button(self.button_start[motor],text='Stop', state='normal',borderwidth=5, bg='green')
                else:                           self.configure_button(self.button_start[motor],text='Start', state='normal',borderwidth=2, bg='green')
                self.button_start[motor].update()
            def open_waveform_dialog(motor=i):
                self.waveform_root.deiconify()
                self.waveform_motor = motor
                waveform_values = [self.rise[motor], self.hold[motor], self.fall[motor], self.freq[motor], self.stretch[motor]]
                self.update_waveform_dialog(waveform_values)
                self.configure_label(self.title_waveform_dialog, text='Row {}'.format(self.well_labels[self.waveform_motor]))
                self.configure_button(self.button_send_waveform, text='Send to Row {}'.format(self.well_labels[motor]))
                self.plot_waveform()
            def move_camera_up(motor=i):
                string = "C{},{}\n".format(motor, self.well_locations[motor]+100)
                self.well_locations[motor]=self.well_locations[motor] + 100
                self.write_to_motors(string)
            def move_camera_down(motor=i):
                string = "C{},{}\n".format(motor, self.well_locations[motor]-100)
                self.well_locations[motor]=self.well_locations[motor] - 100
                self.write_to_motors(string)

            # PLACE FRAME FOR MOTOR
            self.frames[i].configure({  'background':'white', 
                                        'borderwidth':2,
                                        'relief':'groove', 
                                        'width':300, 
                                        'height':100})
            self.frames[i].place(bordermode=tk.OUTSIDE, x=0, y=315-(i*105))

            # ADD BUTTONS
            self.configure_button(self.button_move_to[i],   "Move to Row {}".format(self.well_labels[i]), move_camera, 12, location=[0,0,125,40])
            self.configure_button(self.button_retract[i],   'Retract\nMotor', retract_motor,location=[50,60,50,25], fontsize=8)
            self.configure_button(self.button_start[i],     'Start', enable_motor, location=[0,60,50,25], fontsize=8)
            self.configure_button(self.button_waveform[i],  'Shape', open_waveform_dialog, location=[105,60,50,25], fontsize=8)
            self.button_start[i].configure(self.enable_motor_configuration('stopped'))
            
            self.configure_button(self.button_camera_up[i], "+", move_camera_up, 12, [200, 0, 25, 25])
            self.configure_button(self.button_camera_down[i], "-", move_camera_down, 12, [150, 0, 25, 25])
            
            # ADD LABELS
            self.configure_label(tk.Label(self.frames[i]), ": stretch parameters", fontsize=8, justify=tk.LEFT, location=[160, 60, -1, -1])

        self.waveform_root.protocol("WM_DELETE_WINDOW", self.waveform_window_close)


        # get starting information from arduino

    def send_command(self):
        if len(self.command.get()) > 0:
            self.write_to_motors("{}\n".format(self.command.get()))
            self.command.set('')
            self.entry_command.update()
    
    def toggle_feedback(self):
        if self.feedback_active_flag[self.well_motor]:
            self.feedback_active_flag[self.well_motor] = False
            self.configure_button(self.button_toggle_feedback, text='Toggle Feedback\n(off)', borderwidth=2)
        else:
            self.feedback_active_flag[self.well_motor] = True
            self.configure_button(self.button_toggle_feedback, text='Toggle Feedback\n(on)', borderwidth=5)

    def check_waveform(self):
        self.plot_waveform()
        row = self.well_labels[self.waveform_motor]
        motor = self.waveform_motor
        # CHECK STRETCH
        x = floor(self.stretch[motor].get())
        self.stretch[motor].set(x)
        self.entry_stretch.update()
        input_check = False
        if x > 25:
            printfunc("Row {}: Stretching to more than 25%% is disabled.".format(row))
            self.stretch[motor].set(self.actual_stretch[motor])
            self.entry_stretch.update()
            input_check = False
        elif x > 15: 
            printfunc("Row {}: Setting stretch to {}%% may not be possible.".format(row, x))
            self.actual_stretch[motor] = self.stretch[motor].get()
            input_check = True
        elif x <= 0:
            printfunc("Row {}: Cannot set stretch to {}%%. ".format(row, x))
            self.stretch[motor].set(self.actual_stretch[motor])
            self.entry_stretch.update()
            input_check = False
        else:
            self.actual_stretch[motor] = self.stretch[motor].get()
            input_check = True
        
        # CHECK FREQUENCY
        f = self.freq[motor].get()
        if f > 3:
            printfunc("Row {}: Frequency is too high.  Limiting frequency to 3 Hz.".format(row))
            self.freq[motor].set(3)
            input_check = input_check and True
        elif f <= 0:
            printfunc("Row {}: Frequency cannot be 0 or below.".format(row))
            self.freq[motor].set(self.actual_freq[motor])
            input_check = False

        # CHECK SHAPE
        rise, hold, fall = self.rise[motor].get(), self.hold[motor].get(), self.fall[motor].get()
        if hold < rise + 10 or fall < hold + 10:
            input_check = False
            printfunc('All times must be at least 10 apart.')
        if fall > 90:
            input_check = False
            printfunc('Cannot set fall > 90.')
        if rise < 10:
            input_check = False
            printfunc('Cannot set rise < 10.')
        
        if input_check:
            printfunc("Okay to send to Motor.")
        

    def plot_waveform(self):
        motor = self.waveform_motor
        try:    rise, hold, fall, freq, stretch = self.rise[motor].get(), self.hold[motor].get(), self.fall[motor].get(), self.freq[motor].get(), self.stretch[motor].get()
        except: return
        if hold < rise + 10 or fall < hold + 10 or fall > 90 or rise < 10:
            return
        
        freq = 3 if freq > 3 else freq
        freq = 0.25 if freq <=0 else freq
        t = 1/freq
        x  =[0, rise, hold, fall, 100]
        y = [0, stretch, stretch, 0, 0]
        self.plot_axis.clear()
        self.plot_axis.plot(x,y)
        self.plot_axis.set_xticks(x)
        self.plot_axis.set_xticklabels(["{}\n{}".format(int(x[i]), ['', 'RISE','HOLD','FALL','T={}s'.format(round(t, 2))][i]) for i in range(5)], fontsize=6)
        self.plot_axis.set_yticks([0, stretch])
        self.plot_axis.grid(True)
        self.plot_axis.set_ylim([0, 26])
        self.plot_axis.set_yticklabels(["{}%".format([0, stretch][i]) for i in range(2)], fontsize=6)

        self.plot_canvas.draw()

    def send_waveform_to_all(self):
        for motor in range(4):
            self.waveform_motor=motor
            self.send_waveform()
            self.waveform_root.deiconify()
        self.waveform_root.withdraw()
    
    def send_waveform(self):
        row = self.well_labels[self.waveform_motor]
        motor = self.waveform_motor
        # CHECK STRETCH
        x = floor(self.stretch[motor].get())
        self.stretch[motor].set(x)
        self.entry_stretch.update()
        input_check = False
        if x > 25:
            printfunc("Row {}: Stretching to more than 25%% is disabled.".format(row))
            self.stretch[motor].set(self.actual_stretch[motor])
            self.entry_stretch.update()
            input_check = False
        elif x > 15: 
            printfunc("Row {}: Setting stretch to {}%% may not be possible.".format(row, x))
            self.actual_stretch[motor] = self.stretch[motor].get()
            input_check = True
        elif x <= 0:
            printfunc("Row {}: Cannot set stretch to {}%%. ".format(row, x))
            self.stretch[motor].set(self.actual_stretch[motor])
            self.entry_stretch.update()
            input_check = False
        else:
            self.actual_stretch[motor] = self.stretch[motor].get()
            input_check = True
        
        # CHECK FREQUENCY
        f = self.freq[motor].get()
        if f > 3:
            printfunc("Row {}: Frequency is too high.  Limiting frequency to 3 Hz.".format(row))
            self.freq[motor].set(3)
            input_check = input_check and True
        elif f <= 0:
            printfunc("Row {}: Frequency cannot be 0 or below.".format(row))
            self.freq[motor].set(self.actual_freq[motor])
            input_check = False

        # CHECK SHAPE
        rise, hold, fall = self.rise[motor].get(), self.hold[motor].get(), self.fall[motor].get()
        if hold < rise + 10 or fall < hold + 10:
            input_check = False
            printfunc('All times must be at least 10 apart.')
        if fall > 90:
            input_check = False
            printfunc('Cannot set fall > 90.')
        if rise < 10:
            input_check = False
            printfunc('Cannot set rise < 10.')
        
        # SEND PARAMETERS TO MOTOR
        if input_check:
            self.waveform_root.withdraw()
            string = "S{},{},{},{}\n".format(motor, rise, hold, fall)
            self.write_to_motors(string)
            string = "F{},{}\n".format(motor, f)
            self.write_to_motors(string)
            string = "D{},{}\n".format(motor, int(x*12))
            self.write_to_motors(string)
            printfunc('Setting stretch parameters. ')
        
    def enable_motor_configuration(self, state='disabled'):
        if state == 'disabled':
            configuration = {   'state':'disabled', 
                                'bg':'gray',
                                'borderwidth':2,
                                'text':'Start'}
        elif state == 'stopped':
            configuration = {   'state':'normal', 
                                'bg':'green',
                                'borderwidth':2,
                                'text':'Start'}
        elif state == 'started':
            configuration = {   'state':'normal', 
                                'bg':'green',
                                'borderwidth':5,
                                'text':'Stop'}
        return configuration
    def configure_button(self, button, text=None, command=None, fontsize=None, location=None, **kwargs):
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
    def configure_entry(self, entry, textvariable, location=None, fontsize=None):
        configuration = {'textvariable':textvariable, 'background':'white', 'relief':'groove', 'borderwidth':2}
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
        return entry
    def configure_label(self, label, text, justify=None, fontsize=None, location=None):
        configuration = {'background':'white', 'text':text}
        if fontsize!=None:
            configuration['font'] = font.Font(size=fontsize)
        if justify!=None:
            configuration['justify'] = justify
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
    def update_waveform_dialog(self, values):
        self.entry_rise.configure(textvariable=values[0])
        self.entry_hold.configure(textvariable=values[1])
        self.entry_fall.configure(textvariable=values[2])
        self.entry_freq.configure(textvariable=values[3])
        self.entry_stretch.configure(textvariable=values[4])
        self.title_waveform_dialog.configure(text="Row {}".format(self.well_labels[self.waveform_motor]))
        
    
    def update(self):
        # print(round(time()-self.t0, 1))        
        # if self.waveform_root.winfo_viewable() == 1:
        #     self.plot_waveform()
        # print(time() - self.t0)

        motor_info = self.get_motor_tx()

        if motor_info != None:
            magic = self.extract_magic_number(motor_info)
            self.decode_motor_information(motor_info, magic)
        
            self.update_gui()



        if self.ports.connected_to_camera:
            self.image = self.get_frame_buffer()
            tx_camera = self.get_camera_tx()
            if self.got_image: # got an image in the frame buffer
                self.got_image = False

            if not pyopenmv.script_running(): # stopped running the script
                self.screen.blit(self.screen_font.render("NOT CONNECTED TO CAMERA!", 1, (255, 0, 0)), (0, 0))
                pygame.display.flip()

            # if len(tx_camera) > 0: # frame buffer contains tx info
            #     self.screen.blit(self.screen_font.render("TX: {}".format(tx_camera), 1, (255, 0, 0)), (0, 0))

            pygame.display.flip() # update screen

            self.do_pygame_events()
        
        self.root.after(1, self.update)
        pass

    def update_gui(self):
        for motor in range(4):
            self.button_start[motor].configure( self.get_start_button_configuration_from_motor(motor) )
            if self.well_motor == motor:
                initialized = self.motors[motor].initialized
                if initialized: self.button_home_motor.configure({'borderwidth':5})
                else:           self.button_home_motor.configure({'borderwidth':2})

            # self.button_move_to  = [tk.Button(self.frames[0]),   tk.Button(self.frames[1]),  tk.Button(self.frames[2]),  tk.Button(self.frames[3])]
            # self.button_retract  = [tk.Button(self.frames[0]),   tk.Button(self.frames[1]),  tk.Button(self.frames[2]),  tk.Button(self.frames[3])]
            # self.button_start    = [tk.Button(self.frames[0]),   tk.Button(self.frames[1]),  tk.Button(self.frames[2]),  tk.Button(self.frames[3])]
        
    def get_start_button_configuration_from_motor(self, i):
        motor = self.motors[i]
        enabled = motor.enabled
        if enabled: configuration = self.enable_motor_configuration('started')
        else:       configuration = self.enable_motor_configuration('stopped')
        # print(configuration)
        return configuration
    def decode_motor_information(self, string, magic):
        motor_information = (None, None, None, None)
        if magic == -33: # normal processing
            # print(string.split(';'))
            values = string.split(';')
            if len(values) < 8:
                printfunc('error')
                return 0
            temp = int(values[0])
            t = int(values[1])

            # [2] - tracking information (passthrough from camera)
            tracking_information = values[2].split('&')
            if len(tracking_information) < 7:
                return 0
            
            self.tracking_t = int(tracking_information[0])
            self.tracking_well = int(tracking_information[1])
            self.tracking_motor_homed = bool(int(tracking_information[2]))
            self.tracking_recieved_motor_init_handshake = bool(int(tracking_information[3]))
            self.tracking_stretch = float(tracking_information[4])
            self.tracking_found_max = bool(int(tracking_information[5]))
            self.tracking_maxes = json.loads(tracking_information[6])


            # [3-7] - motor information
            motor_information = values[3:7]
            # print(motor_information)
            for motor in range(4):
                info = motor_information[motor]
                if len(info.split('&')) < 9:
                    return 0
                ID, motor_time, position, distance, frequency, passive_len, homed, camera_initialized, enabled, home_stage = tuple(info.split('&'))
                self.motors[motor].update_motor(
                    t=int(motor_time), 
                    p=int(position), 
                    d=int(distance), 
                    f=float(frequency),
                    enabled= not bool(int(enabled)),
                    homed=bool(int(homed)),
                    initialized=bool(int(camera_initialized)),
                    homing_stage=int(home_stage))
                
            if (time() - self.t_display) > 0.050:
                self.motors[self.well_motor].display()
                self.t_display = time()
                # print("t={}, well={}, homed={}, handshake={}, stretch={}, foundmax={}, maxes={}".format(
                #     self.tracking_t, 
                #     self.tracking_well, 
                #     self.tracking_motor_homed, 
                #     self.tracking_recieved_motor_init_handshake, 
                #     self.tracking_stretch, 
                #     self.tracking_found_max, 
                #     self.tracking_maxes))

                # print('ID {}: enabled={}, '.format(ID, enabled), end='')
            # print('')
            stage_information = values[-1].split('&')
            if len(stage_information) < 6:
                return 0
            
            camera_row, camera_enable, camera_position, camera_well, got_stretch, algorithm_magnet_flag, comm_time = tuple(stage_information)
            try: self.camera_row = int(camera_row)
            except ValueError: self.camera_row = camera_row

            self.camera_enable = bool(int(camera_enable))
            self.camera_position = int(camera_position)
            self.camera_well    = int(camera_well)
            self.got_stretch   = bool(int(got_stretch))
            self.algorithm_magnet_flag = bool(int(algorithm_magnet_flag))
            self.comm_time             = int(comm_time)
            printfunc(self.comm_time)
            # print("{}, {}, {}, {}, {}".format(temp, t, tracking_information, motor_information, stage_information))
            # temp, t, camera_information, motor_information[0], motor_information[1], motor_information[2], motor_information[3], stage_information  = tuple(string.split(';'))
        

    def extract_magic_number(self, string):
        magic = string.split(';')[0]
        try: magic = int(magic)
        except ValueError: magic = -99
        return magic
    def reset_camera_position(self):
        string = "V\n"
        self.write_to_motors(string)
        self.button_move_to[self.well_motor].configure({
            'state':'normal',
            'text':'Move Camera\nto Row {}'.format(self.well_labels[self.well_motor]),
            'borderwidth':2
        })
        self.well_motor = 0
        self.button_move_to[self.well_motor].configure({
            'state':'disabled',
            'text':'Camera Under\nRow {}'.format(self.well_labels[self.well_motor]),
            'borderwidth':7
        })
        printfunc('Reset Camera Position to Row D.')
    def retract_all_motors(self):
        for i in range(4):
            string = "X{}\n".format(i)
            self.write_to_motors(string)
            self.root.after(1)
        printfunc("Retracting all Motors.")
    def stop_all_motors(self):
        for i in range(4):
            string = "O{},0\n".format(i)
            self.write_to_motors(string)
            self.root.after(1)
        printfunc('Stopping all motors.')
    def initialize_camera(self):
        string = 'INIT\n'
        self.write_to_motors(string)
        printfunc('Initializing camera.')
    def deinitialize_camera(self):
        string = "OMV,{}&DEINIT\n".format(self.well_motor)
        self.write_to_motors(string)
    def home_motor(self):
        string = "MOTORINIT&{}\n".format(self.well_motor)
        self.write_to_motors(string)
        printfunc("Homing Motor.")
    def send_adj(self, adj):
        string = "A{},{}\n".format(self.well_motor, adj)
        self.write_to_motors(string)

    def do_pygame_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.destroy()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    self.post_manual = event.pos
                    self.write_to_motors("POSTMANUAL{},{}\n".format(self.post_manual[0], self.post_manual[1]))
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c: # C
                    pygame.image.save(self.image, "{}_capture_{}.png".format(datetime.today().strftime("%Y%m%d_%H%M%S"), self.well_labels[self.well_motor]))
            

    def write_to_motors(self, string):
        if self.conn is not None:
            self.conn.write(string.encode())
        printfunc("{}: {}".format(round(time()-self.t0,1), string[:-1]))
        pass

    def get_frame_buffer(self):
        frame_buffer = pyopenmv.fb_dump()
        if frame_buffer != None:
            self.gotImage = True
            frame = frame_buffer[2]
            size = (frame_buffer[0], frame_buffer[1])
            # create image from RGB888
            image = pygame.image.frombuffer(frame.flat[0:], size, 'RGB')

            # blit stuff
            self.screen.blit(image, (0, 0))
            self.screen.blit(self.bottom_screen_font.render("C{},{}".format(self.camera_well, self.camera_position),1 , (255, 0, 0)), (20, 450))
            self.screen.blit(self.bottom_screen_font.render("D{},{}".format(self.well_motor, self.motors[self.well_motor].d), 1, (0,255,0)), (20,460))
            # self.screen.blit(self.font.render("FPS %.2f"%(fps), 1, (255, 0, 0)), (0, 0))            
            return image
        else:
            return None

    def get_motor_tx(self):
        if self.ports.connected_to_motors:
            string = self.conn.readline().decode('utf-8')[:-2]
            self.conn.flushInput()
        else:
            string = None
        return string
    def get_camera_tx(self):
        # a = pyopenmv.tx_buf_len()
        out = pyopenmv.tx_buf(pyopenmv.tx_buf_len()).decode()[:-2]
        return out

    def waveform_window_close(self):
        self.waveform_root.withdraw()
        self.rise[self.waveform_motor].set(self.actual_rise[self.waveform_motor])
        self.hold[self.waveform_motor].set(self.actual_hold[self.waveform_motor])
        self.fall[self.waveform_motor].set(self.actual_fall[self.waveform_motor])
        self.freq[self.waveform_motor].set(self.actual_freq[self.waveform_motor])
        self.stretch[self.waveform_motor].set(self.actual_stretch[self.waveform_motor])
        
        pass
    def destroy(self):
        self.waveform_root.destroy()
        self.root.destroy()
        pygame.quit()
        if self.ports.connected_to_camera:
            pyopenmv.disconnect()
        sys.exit()

    def run_update(self):
        self.thr = threading.Thread(target=self.update())
        self.thr.start() # run update function in background

    def mainloop(self):
        self.root.mainloop()



def printfunc(string):
    print(string)


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
            if 'OpenMV' in port or 'OpenMV' in desc or 'OpenMV' in hwid:
                continue
            elif 'Bluetooth' in port or 'Bluetooth' in desc or 'Bluetooth' in hwid:
                continue
            else:  
                try:
                    self.conn = Serial(port, baudrate=2000000, timeout=1)
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

if __name__ == "__main__":
    root = CS3D()

    root.run_update()
    root.mainloop()