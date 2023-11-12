from serial import Serial
import tkinter as tk
from tkinter.filedialog import asksaveasfilename, askopenfilename
from tkinter import font
import threading
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import numpy as np
import json, sys, pyopenmv, pygame, os
from math import floor
from datetime import datetime
from serial.tools.list_ports import comports
from time import time
import cv2
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
        
class Protocol:
    def __init__(self):
        self.distance = 300
        self.pre_stretch_delay = 0
        self.stretch_duration=500
        self.hold_duration = 100
        self.unstretch_duration = 500
        self.rest_duration = 3000
        self.n_cycles = 1
        self.ext_trig = 0
        self.assigned_to = [False, False, False, False]
    
    def update(self, distance=None, pre_stretch_delay = None, stretch_duration = None, hold_duration = None, unstretch_duration = None, rest_duration = None, n_cycles = None, ext_trig = None, assigned_to = None):
        if distance             != None: self.distance              = distance
        if pre_stretch_delay    != None: self.pre_stretch_delay     = pre_stretch_delay
        if stretch_duration     != None: self.stretch_duration      = stretch_duration
        if hold_duration        != None: self.hold_duration         = hold_duration
        if unstretch_duration   != None: self.unstretch_duration    = unstretch_duration
        if rest_duration        != None: self.rest_duration         = rest_duration
        if n_cycles             != None: self.n_cycles              = n_cycles
        if ext_trig             != None: self.ext_trig              = ext_trig
        if assigned_to          != None: self.disassigned_totance   = assigned_to
    

class CS3D:
    def __init__(self):
        self.root = tk.Tk()
        self.root.geometry("900x560")
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
        self.screen_size = (648, 480)
        self.screen = pygame.display.set_mode(self.screen_size)
        self.got_image = False
        self.t0 = time()
        self.screen_font = pygame.font.SysFont("monospace", 33)
        self.bottom_screen_font = pygame.font.SysFont("monospace", 14)
        self.t_display = time()

        self.well_labels        = ['D',                         'C',                        'B',                        'A']
        self.well_locations     = [0,                           3900,                       7800,                       11700]
        self.motors             = [Motor(0),                    Motor(1),                   Motor(2),                   Motor(3)]
        self.frames             = [tk.Frame(self.root),         tk.Frame(self.root),        tk.Frame(self.root),        tk.Frame(self.root)]
        self.button_move_to     = [tk.Button(self.frames[0]),   tk.Button(self.frames[1]),  tk.Button(self.frames[2]),  tk.Button(self.frames[3])]
        self.button_retract     = [tk.Button(self.frames[0]),   tk.Button(self.frames[1]),  tk.Button(self.frames[2]),  tk.Button(self.frames[3])]
        self.button_start       = [tk.Button(self.frames[0]),   tk.Button(self.frames[1]),  tk.Button(self.frames[2]),  tk.Button(self.frames[3])]
        self.button_waveform    = [tk.Button(self.frames[0]),   tk.Button(self.frames[1]),  tk.Button(self.frames[2]),  tk.Button(self.frames[3])]
        self.button_camera_up   = [tk.Button(self.frames[0]),   tk.Button(self.frames[1]),  tk.Button(self.frames[2]),  tk.Button(self.frames[3])]
        self.button_camera_down = [tk.Button(self.frames[0]),   tk.Button(self.frames[1]),  tk.Button(self.frames[2]),  tk.Button(self.frames[3])]
        
        

        self.global_t = 0
        self.start_t = 0
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
        self.camera_positioSn = 0
        self.camera_well = 0
        self.running_processing = True
        self.save_image_as_video = False
        self.videowriter = None
        self.time_of_recording = ''
        self.image_size = (640,480)
        self.got_stretch   = False
        self.algorithm_magnet_flag = False

        self.x1, self.x2, self.y1, self.y2 = 0,0,0,0
        self.rect_making = False
        self.make_a_rect = False

        self.post_manual = (53,125)

        self.ttl_received = 0
        self.ttl_mode = 0
        self.n_ttl = 0

        self.frame_rate = 30
        self.tracking_t = 0
        # self.waveform_root.protocol("MW_DELETE_WINDOW", self.waveform_window_close)
        
        # CONFIGURE GLOBAL COMMAND BUTTONS
        self.configure_global_commands()

        
        # CONFIGURE EACH ROW
        for i in range(4):
            self.configure_row(i)
            
        self.protocol = Protocol()
        self.passed_check_protocol = False
        self.protocol_display_message = ''
        self.protocol_running = False
        self.EXTERNAL_TRIGGER_FLAG = False
        self.configure_protocol_frame()
        
        # get starting information from arduino
        motor_info = self.get_motor_tx()
        magic = self.extract_magic_number(motor_info)
        ret = self.decode_motor_information(motor_info, magic)
        
        self.start_t = self.global_t    





    def send_command(self):
        if len(self.command.get()) > 0:
            self.last_command = self.command.get()
            self.write_to_motors("{}\n".format(self.command.get()))
            self.command.set('')
            self.entry_command.update()

            with open('past_commands.txt', 'a') as f:
                f.write("{}: {}\n".format(datetime.today().strftime("%m/%d/%Y %H:%M:%S"), self.last_command))

    def toggle_feedback(self):
        if self.feedback_active_flag[self.well_motor]:
            self.feedback_active_flag[self.well_motor] = False
            self.configure_button(self.button_toggle_feedback, text='Toggle Feedback\n(off)', borderwidth=2)
        else:
            self.feedback_active_flag[self.well_motor] = True
            self.configure_button(self.button_toggle_feedback, text='Toggle Feedback\n(on)', borderwidth=5)

    def configure_row(self, i):
        def move_camera(motor=i):
            string = "C{},{}\n".format(motor, self.well_locations[motor])
            current_filename = self.name_var.get()

            self.name_var.set(self.well_labels[motor] + current_filename[1:])
            self.write_to_motors(string)
            self.button_move_to[self.well_motor].configure(text='Move Camera\nto Row {}'.format(self.well_labels[self.well_motor]))
            self.button_move_to[self.well_motor].configure(borderwidth=2)
            self.button_move_to[self.well_motor].configure(state='normal')
            self.well_motor = motor
            self.button_move_to[self.well_motor].configure(text='Camera Under\nRow {}'.format(self.well_labels[self.well_motor]))
            self.button_move_to[self.well_motor].configure(borderwidth=7)
            self.button_move_to[self.well_motor].configure(state='disabled')
            
            self.frame_initialization.place(x=330, y=315-(motor*105))
            if self.motors[self.well_motor].initialized == True:
                self.configure_button(self.button_initialize_camera, 'Reset\nCamera')
            else:
                self.configure_button(self.button_initialize_camera, 'Initialize\nCamera')
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
        self.configure_button(self.button_move_to[i],   "Move to Row {}".format(self.well_labels[i]), move_camera, 12, location=[0,0,145,50])
        self.configure_button(self.button_retract[i],   'Retract Motor', retract_motor,location=[155,50,135,40], fontsize=12)
        self.configure_button(self.button_start[i],     'Start', enable_motor, location=[155,0,135,40], fontsize=14)
        # self.configure_button(self.button_waveform[i],  'Shape', open_waveform_dialog, location=[105,60,50,25], fontsize=8)
        self.button_start[i].configure(self.enable_motor_configuration('stopped'))
        
        self.configure_button(self.button_camera_up[i], "+", move_camera_up, 12, [75, 70, 25, 25])
        self.configure_button(self.button_camera_down[i], "-", move_camera_down, 12, [25, 70, 25, 25])
        
        # ADD LABELS
        self.configure_label(tk.Label(self.frames[i]), "Camera Adjust: ", fontsize=8, anchor='e', location=[25, 50, -1, -1])


    def configure_global_commands(self):
        self.button_reset_camera_position   = self.configure_button(tk.Button(self.root), "Reset Camera\nPosition", self.reset_camera_position, 10, [100, 430, 100, 50])
        self.button_retract_all_motors      = self.configure_button(tk.Button(self.root), "Retract All\nMotors",    self.retract_all_motors,    10, [200, 430, 100, 50])
        self.button_stop_all_motors         = self.configure_button(tk.Button(self.root), "Stop All\nMotors",       self.stop_all_motors,       10, [0, 430, 100, 50])
        self.button_teardown                = self.configure_button(tk.Button(self.root), "Teardown\nMotors",       self.teardown_motors,       10, [200,480,100,50])

        self.frame_initialization           = tk.Frame(self.root)
        self.button_initialize_camera       = self.configure_button(tk.Button(self.frame_initialization), "Initialize\nCamera",     self.initialize_camera,     10, [20, 0, 100, 40])
        self.button_home_motor              = self.configure_button(tk.Button(self.frame_initialization), "Home Motor",             self.home_motor,            10, [20, 40, 100, 25])
        self.button_adj_up                  = self.configure_button(tk.Button(self.frame_initialization), "-",                      lambda: self.send_adj(0),   10, [0, 42.5, 20, 20])
        self.button_adj_down                = self.configure_button(tk.Button(self.frame_initialization), "+",                      lambda: self.send_adj(1),   10, [120, 42.5, 20, 20])
        self.button_record_video            = self.configure_button(tk.Button(self.frame_initialization), "Record\nVideo",          self.toggle_video_recording,10, [20, 65, 100, 40])
        # self.button_toggle_feedback         = self.configure_button(tk.Button(self.frame_initialization), "Toggle Feedback\n(off)", command=self.toggle_feedback, fontsize=10, location=[10, 110, 120, 40])
        
        self.name_var = tk.StringVar(self.frame_initialization, 'D')
        self.plate_name_var = tk.StringVar(self.frame_initialization, 'PLT001')
        
        self.full_filename = ''
        self.configure_label(tk.Label(self.frame_initialization), 'Filename: ', location=[0, 105, 150, 25], fontsize=10,anchor='w')
        self.configure_entry(tk.Entry(self.frame_initialization), self.name_var,         [0,130,150,25], 10)
        
        self.configure_label(tk.Label(self.frame_initialization), "Plate: ", location=[0,155,150, 25], fontsize=10, anchor='w')
        self.configure_entry(tk.Entry(self.frame_initialization), self.plate_name_var, [0,180,150,25], 10)
        
        self.entry_command                  = self.configure_entry(tk.Entry(self.root),     textvariable=self.command, location=[10, 500, -1, -1], fontsize=10 )
        self.entry_command.bind('<Return>', func=lambda val: self.send_command())
        self.entry_command.bind


        self.frame_initialization.configure({  'background':'white', 
                                        'borderwidth':0,
                                        'relief':'groove', 
                                        'width':150, 
                                        'height':250})
        self.frame_initialization.place(x=330, y=315)

    def configure_protocol_frame(self):
        self.protocol_frame = tk.Frame(self.root)
        self.protocol_buttons = [tk.Button(self.protocol_frame), tk.Button(self.protocol_frame), tk.Button(self.protocol_frame), tk.Button(self.protocol_frame)]
        self.protocol_assigned_to_motors = [              False,                          False,                          False,                          False]
        self.configure_label(tk.Label(self.protocol_frame), 'PROTOCOL', 14, [0, 0, 175, 30])

        self.var_distance = tk.StringVar(self.protocol_frame,300)
        self.var_pre_stretch_delay = tk.StringVar(self.protocol_frame,0)
        self.var_stretch_duration = tk.StringVar(self.protocol_frame,500)
        self.var_hold_duration = tk.StringVar(self.protocol_frame,100)
        self.var_fall_duration = tk.StringVar(self.protocol_frame, 500)
        self.var_rest_duration = tk.StringVar(self.protocol_frame, 3000)
        self.var_n_cycles = tk.StringVar(self.protocol_frame, 1)

        self.configure_label(tk.Label(self.protocol_frame), 'Motor distance (steps): ', 12, [0, 30, 200, 25], anchor='e')
        self.configure_label(tk.Label(self.protocol_frame), 'pre-stretch delay (ms): ', 12, [0, 55, 200, 25], anchor='e') #, self.configure_label(tk.Label(self.protocol_frame), '>=100', 10, [250, 55, 200, 25], anchor='w')
        self.configure_label(tk.Label(self.protocol_frame), 'stretch duration (ms): ', 12, [0, 80, 200, 25], anchor='e')
        self.configure_label(tk.Label(self.protocol_frame), 'hold duration (ms): ', 12, [0, 105, 200, 25], anchor='e'), self.configure_label(tk.Label(self.protocol_frame), '>=100', 10, [250, 105, 200, 25], anchor='w')
        self.configure_label(tk.Label(self.protocol_frame), 'un-stretch duration (ms): ', 12, [0, 130, 200, 25], anchor='e')
        self.configure_label(tk.Label(self.protocol_frame), 'rest duration (ms): ', 12, [0, 155, 200, 25], anchor='e'), self.configure_label(tk.Label(self.protocol_frame), '>=100', 10, [250, 155, 200, 25], anchor='w')
        self.configure_label(tk.Label(self.protocol_frame), 'N cycles (-1 for inf): ', 12, [0, 180, 200, 25], anchor='e')
        
        self.label_speed_rise = self.configure_label(tk.Label(self.protocol_frame), "speed = {} steps/s".format(self.motors[self.well_motor].speed_rise), 10, [250, 80, 200, 25], anchor='w')
        self.label_speed_fall = self.configure_label(tk.Label(self.protocol_frame), "speed = {} steps/s".format(self.motors[self.well_motor].speed_fall), 10, [250, 130, 200, 25], anchor='w')

        self.entry_distance = self.configure_entry(tk.Entry(self.protocol_frame), self.var_distance, [200, 30, 50, 25])
        self.entry_pre_stretch_delay = self.configure_entry(tk.Entry(self.protocol_frame), self.var_pre_stretch_delay, [200, 55, 50, 25])
        self.entry_stretch_duration = self.configure_entry(tk.Entry(self.protocol_frame), self.var_stretch_duration, [200, 80, 50, 25])
        self.entry_hold_duration = self.configure_entry(tk.Entry(self.protocol_frame), self.var_hold_duration, [200, 105, 50, 25])
        self.entry_fall_duration = self.configure_entry(tk.Entry(self.protocol_frame), self.var_fall_duration, [200, 130, 50, 25])
        self.entry_rest_duration = self.configure_entry(tk.Entry(self.protocol_frame), self.var_rest_duration, [200, 155, 50, 25])
        self.entry_n_cycles = self.configure_entry(tk.Entry(self.protocol_frame), self.var_n_cycles, [200, 180, 50, 25])

        
        def toggleExtTrig():
            self.EXTERNAL_TRIGGER_FLAG = not self.EXTERNAL_TRIGGER_FLAG

            if self.EXTERNAL_TRIGGER_FLAG:
                self.button_trig_toggle.configure(borderwidth=5)
            else:
                self.button_trig_toggle.configure(borderwidth=2)
            pass
        self.button_trig_toggle = self.configure_button(tk.Button(self.protocol_frame), "External\ntrigger", toggleExtTrig, 10, [260, 180, 60, 40])
        self.button_check_protocol = self.configure_button(tk.Button(self.protocol_frame), 'Verify Protocol', self.verify_protocol, 12, [100, 220, 150, 25])
        # self.button_save_protocol  = self.configure_button(tk.Button(self.protocol_frame), 'Save', self.save_protocol, 12, [250, 220, 60, 25])

        self.configure_label(tk.Label(self.protocol_frame), "Assign to:", 11, [0, 250, 125, 25], 'e' )
        for i in range(4):
            self.protocol_buttons[i] = tk.Button(self.protocol_frame)
            def send_protocol(motor=i):
                temp = int(self.var_n_cycles.get())

                string = "PROTOCOL{well},{distance},{pre_stretch_delay},{stretch_duration},{hold_duration},{fall_duration},{rest_duration},{n_cycles},{trig}\n".format(
                    well=motor,
                    distance=int(self.var_distance.get()),
                    pre_stretch_delay = int(self.var_pre_stretch_delay.get()),
                    stretch_duration = int(self.var_stretch_duration.get()),
                    hold_duration = int(self.var_hold_duration.get()),
                    fall_duration = int(self.var_fall_duration.get()),
                    rest_duration = int(self.var_rest_duration.get()),
                    n_cycles = temp,
                    trig = int(self.EXTERNAL_TRIGGER_FLAG))
            
                self.update_protocol_display()
                if self.passed_check_protocol:
                    self.motors[motor].assigned_protocol = not self.motors[motor].assigned_protocol
                    if self.motors[motor].assigned_protocol:
                        self.protocol_buttons[motor].configure(borderwidth=2)
                        self.protocol_assigned_to_motors[motor] = False
                        self.protocol.assigned_to[motor] = False
                        # self.write_to_motors(string)
                        self.protocol_display_message += 'Unassigned to Row {}.\n'.format(self.well_labels[motor])
                        self.update_protocol_display()


                    else:
                        self.protocol_buttons[motor].configure(borderwidth=5)
                        self.protocol_assigned_to_motors[motor] = True
                        self.write_to_motors(string)
                        self.protocol_display_message += 'Assigned to Row {}.\n'.format(self.well_labels[motor])
                        self.update_protocol_display()
                        self.protocol.assigned_to[motor] = True

                    # self.protocol_display.configure(text='Sent to Row {well}'.format(self.well_labels[self.well_motor]))
                else:
                    self.protocol_display_message = "Please verify protocol."
                    self.update_protocol_display()
                    pass
                pass
            self.configure_button(self.protocol_buttons[i],self.well_labels[i], send_protocol, 12, [200-(i*25), 250, 25, 25])

        self.button_begin_protocols = self.configure_button(tk.Button(self.protocol_frame), "Start Protocol", self.begin_protocol, 14, [100, 280, 150, 30])

        self.label_protocol_begin = self.configure_label(tk.Label(self.protocol_frame), "", 10, [250, 280, 150, 70], anchor='nw')

        self.protocol_display = self.configure_label(tk.Label(self.protocol_frame), "", 12, [0, 310, 500, 300], anchor='nw', justify=tk.LEFT)

        self.protocol_frame.configure({'background':'white', 
                                        'borderwidth':0,
                                        'relief':'groove', 
                                        'width':500, 
                                        'height':500})
        self.protocol_frame.place(bordermode=tk.OUTSIDE, x=500, y=10)

    def begin_protocol(self):
        string = "E{}{}{}{}\n".format(int(self.protocol_assigned_to_motors[0]),
                                      int(self.protocol_assigned_to_motors[1]),
                                      int(self.protocol_assigned_to_motors[2]),
                                      int(self.protocol_assigned_to_motors[3]))
        self.write_to_motors(string)

        self.root.after(1)

        enabled = [0,0,0,0]
        for i in range(4):
            enabled[i] = self.motors[i].enabled
        
        print(enabled)
        print(self.protocol_assigned_to_motors)
        
        if enabled == self.protocol_assigned_to_motors:
            self.protocol_running = False
        else:
            self.protocol_running = True
        # self.protocol_running = not self.protocol_running

        
        if self.protocol_running:
            self.button_begin_protocols.configure(text='Stop Protocol', borderwidth=5)
            self.n_ttl = 0 # reset n_ttl count
        else:
            self.button_begin_protocols.configure(text='Start Protocol', borderwidth=2)
            
        pass
    def update_protocol_display(self):
        if self.protocol_display_message.count('\n') > 8:
            self.protocol_display_message = self.protocol_display_message[(self.protocol_display_message.index('\n')+1):]

        self.protocol_display.configure(text=self.protocol_display_message)
        pass

    def verify_protocol(self):
        self.protocol_display_message = ''
        self.passed_check_protocol = True
        try:
            distance = int(self.var_distance.get())
            pre_stretch_delay = int(self.var_pre_stretch_delay.get())
            stretch_duration = int(self.var_stretch_duration.get())
            hold_duration = int(self.var_hold_duration.get())
            fall_duration = int(self.var_fall_duration.get())
            rest_duration = int(self.var_rest_duration.get())
            n_cycles = int(self.var_n_cycles.get())
        except:
            self.passed_check_protocol = False
            self.protocol_display_message += 'Cannot get number from above.  Check values.'
            self.update_protocol_display()
            return
        
        
        # if distance/stretch_duration > 4:
        #     self.passed_check_protocol = False
        #     self.protocol_display_message += '- stretch motor speed too large (>4000 steps/s)\n'
        #     self.protocol_display_message += '  - current speed = {} steps/s\n'.format(1000*distance/stretch_duration)
        #     self.protocol_display_message += '  - reduce distance to {} steps, or\n  - increase duration to {} ms\n'.format(stretch_duration*4, distance/4)
        # if distance/fall_duration > 4:
        #     self.passed_check_protocol = False
        #     self.protocol_display_message += '- unstretch motor speed too large (>4000 steps/s)\n'
        #     self.protocol_display_message += '  - current speed = {} steps/s\n'.format(1000*distance/fall_duration)
        #     self.protocol_display_message += '  - reduce distance to {} steps, or\n  - increase duration to {} ms\n'.format(fall_duration*4, distance/4)
        
        
        # if pre_stretch_delay < 100:
        #     self.passed_check_protocol = False
        #     self.protocol_display_message += '- pre-stretch delay too small (< 100 ms)\n'
        if hold_duration < 100:
            self.passed_check_protocol = False
            self.protocol_display_message += '- hold duration too small (< 100 ms)\n'
        if rest_duration < 100:
            self.passed_check_protocol = False
            self.protocol_display_message += '- rest duration too small (< 100 ms)\n'
        
        if n_cycles < -1 or n_cycles == 0:
            self.passed_check_protocol = False
            self.protocol_display_message += '- number of cycles must be >=1 or -1 for inf\n'
        
        
        if self.passed_check_protocol == True:
            self.label_speed_rise.configure(text="speed = {} steps/s".format(1000*distance/stretch_duration))
            self.label_speed_fall.configure(text="speed = {} steps/s".format(1000*distance/fall_duration))
             
            self.protocol_display_message += 'Okay to send.\n'
            tot_duration = pre_stretch_delay + stretch_duration + hold_duration + fall_duration + rest_duration
            tot_stretch_duration = stretch_duration + hold_duration + fall_duration
            self.protocol_display_message += '- Approximate protocol duration: {} ms.\n'.format(tot_duration)
            self.protocol_display_message += '- Approximate total stretch duration: {} ms.\n'.format(tot_stretch_duration)
            self.protocol_display_message += '- Approximate frequency of stretching: {} Hz.\n'.format(round(1/float(tot_duration/1000),3))
            if self.EXTERNAL_TRIGGER_FLAG:
                if n_cycles == 1:
                    self.protocol_display_message += '- External trigger. One stretch per trigger.\n'
                else:
                    if n_cycles == -1:
                        n_cycles = 'inf'
                    self.protocol_display_message += '- External trigger. \n   - WARNING: Number of stretching cycles = {}.\n'.format(n_cycles)
                
            else:
                self.protocol_display_message += '- Open Loop. Number of cycles = {}.\n'.format(n_cycles)


            for i in range(4):
                self.protocol_assigned_to_motors[i] = False
                self.protocol.assigned_to[i] = False
                self.motors[i].assigned_protocol = True
                self.protocol_buttons[i].configure(borderwidth=2)

            self.protocol.distance = distance
            self.protocol.pre_stretch_delay = pre_stretch_delay
            self.protocol.stretch_duration = stretch_duration
            self.protocol.hold_duration = hold_duration
            self.protocol.unstretch_duration = fall_duration
            self.protocol.rest_duration = rest_duration
            self.protocol.n_cycles = n_cycles
            self.protocol.ext_trig = self.EXTERNAL_TRIGGER_FLAG

            
            
        self.update_protocol_display()
        pass
    
    def save_protocol(self):
        root2 = tk.Tk()
        root2.withdraw()

        savefilename = asksaveasfilename(initialdir=os.getcwd(),  defaultextension='.json')
        print(savefilename)
        if savefilename is None:
            return
        
        

        return
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
    def configure_label(self, label, text, fontsize=None, location=None, anchor=None, **kwargs):
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
    def update_waveform_dialog(self, values):
        self.entry_rise.configure(textvariable=values[0])
        self.entry_hold.configure(textvariable=values[1])
        self.entry_fall.configure(textvariable=values[2])
        self.entry_freq.configure(textvariable=values[3])
        self.entry_stretch.configure(textvariable=values[4])
        self.title_waveform_dialog.configure(text="Row {}".format(self.well_labels[self.waveform_motor]))
        
    
    def update(self):
        self.t = time() - self.t0
        # print(self.frame_rate)
        # print(round(time()-self.t0, 1))        
        # if self.waveform_root.winfo_viewable() == 1:
        #     self.plot_waveform()
        # print(time() - self.t0)

        motor_info = self.get_motor_tx()

        if motor_info != None:
            magic = self.extract_magic_number(motor_info)
            ret = self.decode_motor_information(motor_info, magic)
            self.update_gui()

            print("{:.1f}\t".format(self.global_t/1000 - self.start_t/1000), end='')
            self.motors[self.well_motor].display()
            if self.ttl_mode:
                temp = " TTL"
            else:
                temp = "OPEN"
            printfunc("{}, trig: {}, n_TRIG: {}".format(temp, self.ttl_received, self.n_ttl), end='\r')
            # print(motor_info)

        if self.ports.connected_to_camera:
            self.image = self.get_frame_buffer()
            tx_camera = self.get_camera_tx()
            if self.got_image: # got an image in the frame buffer
                self.got_image = False

            if not pyopenmv.script_running(): # stopped running the script
                self.screen.blit(self.screen_font.render("ERROR.  RESTARTING, CAMERA.", 1, (255, 0, 0)), (0, 100))
                pygame.display.flip()
                # attempt to restart openmv
                pyopenmv.disconnect()
                t0 = time()
                while time() < t0+3:
                    pass
                self.ports.connect_to_camera()

                

            if self.make_a_rect:
                top_left = (min(self.x1, self.x2), min(self.y1, self.y2))
                top_right = (max(self.x1, self.x2), min(self.y1, self.y2))
                bottom_left = (min(self.x1,self.x2), max(self.y1, self.y2))
                bottom_right = (max(self.x1,self.x2), max(self.y1, self.y2))
                
                color = (255, 255, 0)
                
                pygame.draw.line(self.screen, color, top_left, top_right, 2) # left
                pygame.draw.line(self.screen, color, top_left, bottom_left, 2) # bottom
                pygame.draw.line(self.screen, color, top_right, bottom_right, 2) # right
                pygame.draw.line(self.screen, color, bottom_left, bottom_right, 2) # top
             
            # if len(tx_camera) > 0: # frame buffer contains tx info
            #     self.screen.blit(self.screen_font.render("TX: {}".format(tx_camera), 1, (255, 0, 0)), (0, 0))
             
            pygame.display.flip() # update screen

        self.do_pygame_events()

         
            
        # pygame.draw.rect(screen, (255,0,0), rectangle)
             
            # pygame.draw.rect(screen, (255,255,255), pygame.rect.Rect(min(x1,x2), min(y1,y2), abs(x1-x2), abs(y1-y2)))
        
        t2 = time() - self.t0
        if t2 > self.t:
            self.frame_rate = 1/(t2-self.t)

        self.root.after(1, self.update)
        
        pass

    def update_gui(self):
        for motor in range(4):
            self.button_start[motor].configure( self.get_start_button_configuration_from_motor(motor) )
            if self.well_motor == motor:
                initialized = self.motors[motor].initialized
                if initialized: self.button_home_motor.configure({'borderwidth':5})
                else:           self.button_home_motor.configure({'borderwidth':2})

        # if self.update_label == True:
        if self.motors[self.well_motor].enabled:
            if self.ttl_mode == 1:
                if self.ttl_received == 0:
                    self.label_protocol_begin.configure(text = "Waiting for trigger...")
                    # self.update_label = False
                else:
                    self.label_protocol_begin.configure(text = "Running stretch protocol.")
                    # self.update_label = False
            else:
                self.label_protocol_begin.configure(text = "Running stretch protocol\n(Open loop).")
                # self.update_label = False
        else:
            self.label_protocol_begin.configure(text = '')
                # self.update_label = False
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
                return -1
            temp = int(values[0])
            self.global_t = int(values[1])

            # [2] - tracking information (passthrough from camera)
            tracking_information = values[2].split('&')
            # print(tracking_information)
            if len(tracking_information) < 8:
                return -1
            
        
            self.tracking_t = int(tracking_information[0])
            self.tracking_well = int(tracking_information[1])
            self.tracking_motor_homed = bool(int(tracking_information[2]))
            self.tracking_recieved_motor_init_handshake = bool(int(tracking_information[3]))
            self.tracking_stretch = float(tracking_information[4])
            self.tracking_found_max = bool(int(tracking_information[5]))
            # self.tracking_maxes = json.loads(tracking_information[6])
            self.magic = tracking_information[7]
            # print(tracking_information)
            # [3-7] - motor information
            motor_information = values[3:7]
            # print(motor_information)
            for motor in range(4):
                info = motor_information[motor]
                if len(info.split('&')) < 9:
                    return -1
                ID, motor_time, position, distance, t_delay, t_rise, t_hold, t_fall, period, passive_len, homed, camera_initialized, enabled, cycle_count, n_cycles = tuple(info.split('&'))

                # ID, motor_time, position, distance, speed_rise, speed_fall, passive_len, homed, enabled, cycle_count, n_cycles = tuple(info.split('&'))
                # print(info.split('&'))
                self.motors[motor].update_motor(
                    t=int(motor_time), 
                    p=int(position), 
                    d=int(distance), 
                    speed_rise = float(1e6*float(distance) / (float(t_rise)-float(t_delay))),
                    speed_fall = float(1e6*float(distance) / (float(t_fall)-float(t_hold))),
                    enabled= not bool(int(enabled)),
                    homed=bool(int(homed)),
                    initialized=bool(int(camera_initialized)),
                    # homing_stage=int(home_stage),
                    cycle_count = int(cycle_count),
                    n_cycles = int(n_cycles),
                    ts = [int(int(i)/1000) for i in [t_delay, t_rise, t_hold, t_fall, period]]
                    )
                
            if (time() - self.t_display) > 0.050:
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
                return -1
            
            camera_row, camera_enable, camera_position, camera_well, got_stretch, algorithm_magnet_flag, ttl_recieved, ttl_mode, comm_time = tuple(stage_information)
            # print(stage_information)
            try: self.camera_row = int(camera_row)
            except ValueError: self.camera_row = camera_row

            self.camera_enable = bool(int(camera_enable))
            self.camera_position = int(camera_position)
            self.camera_well    = int(camera_well)
            self.got_stretch   = bool(int(got_stretch))
            self.algorithm_magnet_flag = bool(int(algorithm_magnet_flag))
            self.comm_time             = int(comm_time)

            if int(ttl_recieved) == 1 and self.ttl_received == 0:
                self.n_ttl += 1
            

            self.ttl_received = int(ttl_recieved)
            self.ttl_mode = int(ttl_mode)
            
            # printfunc("{}: {}".format(t, self.comm_time))
            # print("{}, {}, {}, {}, {}".format(temp, t, tracking_information, motor_information, stage_information))
            # temp, t, camera_information, motor_information[0], motor_information[1], motor_information[2], motor_information[3], stage_information  = tuple(string.split(';'))
            return 0
        return -1

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

    def teardown_motors(self):
        self.write_to_motors("P1111,3000\n")
        self.root.after(1)
    def initialize_camera(self):
        if self.motors[self.well_motor].initialized == False:
            string = 'INIT\n'
            printfunc('Initializing camera.')
            self.configure_button(self.button_initialize_camera, 'Reset\nCamera')
        else:
            string = "OMV,{}&DEINIT\n".format(self.well_motor)
            printfunc('Uninitializing camera.')
            self.configure_button(self.button_initialize_camera, 'Initialize\nCamera')
        self.write_to_motors(string)
    def toggle_video_recording(self):
        self.running_processing = not self.running_processing
        self.save_image_as_video = not self.save_image_as_video
        string = "OMV,{}&REMOVEPROCESSING\n".format(self.well_motor)
        print(self.save_image_as_video)
        if self.save_image_as_video:
            self.configure_button(self.button_record_video, 'Recording', borderwidth=5)
            self.time_of_recording = datetime.today().strftime("%Y%m%d_%H%M%S")
            self.video_filename = 'data/{}_{}_videocapture_{}.avi'.format(self.plate_name_var.get(), self.name_var.get(), self.time_of_recording)
            self.txt_filename = "data/{}_{}_video_information_{}.txt".format(self.plate_name_var.get(), self.name_var.get(), self.time_of_recording)
            self.videowriter = cv2.VideoWriter(self.video_filename, 
                         cv2.VideoWriter_fourcc('I','4','2','0'),
                         self.frame_rate, self.image_size)
            with open(self.txt_filename, 'w') as f:
                f.write('global time (ms),well,motor time (ms),motor position (steps),max distance (steps),cycle count,TrigMode,TRIG,post X (px),post Y (px)\n')
                
            
            
            
        else:
            self.configure_button(self.button_record_video, 'Record\nVideo', borderwidth=2)
            if self.videowriter is not None:
                try:    self.videowriter.release()
                except: pass
                self.videowriter = None
            

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
            # print(event)
            if event.type == pygame.QUIT:
                self.destroy()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    if not self.motors[self.well_motor].enabled:
                        self.post_manual = event.pos
                        self.write_to_motors("POSTMANUAL{},{}\n".format(self.post_manual[0], self.post_manual[1]))

                if event.button == 3:
                    if event.button == 3:
                        self.make_a_rect = True
                        if self.rect_making == False:
                            self.x1, self.y1 = event.pos
                            self.x2, self.y2 = event.pos
                            self.rect_making = True
                    
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 3:
                    self.rect_making = False
                    self.make_a_rect = False
                    if self.x1 != self.x2 and self.y1 != self.y2:
                        self.write_to_motors("OMV,{}&ROI&{}&{}&{}&{}#\n".format(self.camera_well, min(self.x1, self.x2), max(self.x1, self.x2), min(self.y1, self.y2), max(self.y1, self.y2)))
                    

            elif event.type == pygame.MOUSEMOTION:
                if self.rect_making:
                    self.x2, self.y2 = event.pos
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c: # C
                    pygame.image.save(self.image, "{}_capture_{}.png".format(self.well_labels[self.well_motor], datetime.today().strftime("%Y%m%d_%H%M%S")))
            

    def write_to_motors(self, string):
        if self.conn is not None:
            self.conn.write(string.encode())
            
            printfunc("{}: {}\n".format(round(time()-self.t0,1), string[:-1]))
        pass

    def get_frame_buffer(self):
        frame_buffer = pyopenmv.fb_dump()
        if frame_buffer != None:

            self.gotImage = True
            frame = frame_buffer[2]
            size = (frame_buffer[0], frame_buffer[1])
            self.image_size = size
            # create image from RGB888
            if self.screen_size != self.image_size:
                pygame.display.set_mode((self.image_size))
                self.screen_size = self.image_size
            image = pygame.image.frombuffer(frame.flat[0:], size, 'RGB')
            # print(self.videowriter)

            # blit stuff
            self.screen.blit(image, (0, 0))
            self.screen.blit(self.bottom_screen_font.render("C{},{}".format(self.camera_well, self.camera_position),1 , (255, 0, 0)), (20, self.screen_size[1]-40))
            self.screen.blit(self.bottom_screen_font.render("D{},{}".format(self.well_motor, self.motors[self.well_motor].d), 1, (0,255,0)), (20,self.screen_size[1]-30))
            self.screen.blit(self.bottom_screen_font.render("P{}".format(self.motors[self.well_motor].p), 1, (255, 0, 255)), (20, self.screen_size[1]-20))
            
            if self.save_image_as_video:
            # if self.videowriter is not None:
                self.videowriter.write(frame)
                with open(self.txt_filename, 'a') as f:

                    f.write('{t},{well},{motor_t},{motor_p},{motor_d},{cycle_count},{ttl_mode},{ttl_received},{n_ttl},{post_x},{post_y},\n'.format(
                        t = round((time() - self.t0)*1000, 6), 
                        well = self.well_labels[self.well_motor], 
                        motor_t = self.motors[self.well_motor].t,
                        motor_p = self.motors[self.well_motor].p, 
                        motor_d = self.motors[self.well_motor].d,
                        cycle_count = self.motors[self.well_motor].cycle_count,
                        ttl_mode = self.ttl_mode,
                        ttl_received = self.ttl_received,
                        n_ttl = self.n_ttl,
                        post_x = self.post_manual[0],
                        post_y = self.post_manual[1]))
            
            
            # self.screen.blit(self.font.render("FPS %.2f"%(fps), 1, (255, 0, 0)), (0, 0))            
            return image
        else:
            return None

    def get_motor_tx(self):
        if self.ports.connected_to_motors:
            string = self.conn.readline()
            # print(string)
            try:
                string = string.decode('utf-8')[:-2]
            except:
                string = None
            self.conn.reset_input_buffer()

        else:
            string = None
        return string
    def get_camera_tx(self):
        # a = pyopenmv.tx_buf_len()
        out = pyopenmv.tx_buf(pyopenmv.tx_buf_len()).decode()[:-2]
        return out

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

if __name__ == "__main__":
    root = CS3D()

    root.run_update()
    root.mainloop()