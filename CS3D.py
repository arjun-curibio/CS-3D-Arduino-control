from serial import Serial
import tkinter as tk
from tkinter.filedialog import asksaveasfilename, askopenfilename, askdirectory
from tkinter import font
import threading
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import json, sys, pyopenmv, pygame, os
from datetime import datetime
from serial.tools.list_ports import comports
from time import time
import cv2
from pathlib import Path

from CS3D_aux import *

class CS3D:
    def __init__(self):
        self.root = tk.Tk()
        self.root.geometry("1300x560")
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
        self.camera_position = 0
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
        self.recording = False
        self.ttl_received = 0
        self.ttl_mode = 0
        self.n_ttl = 0

        self.monitor_ttl_for_recording = False
        self.got_ttl = False
        self.frame_rate = 30
        self.last_framerates = []
        self.tracking_t = 0

        # self.waveform_root.protocol("MW_DELETE_WINDOW", self.waveform_window_close)
        
        # CONFIGURE GLOBAL COMMAND BUTTONS

        
        # CONFIGURE EACH ROW
        for i in range(4):
            self.configure_row(i)
        self.configure_global_commands()

        # self.protocol = Protocol()
        

        
        self.DEFAULT_PARENT_FOLDER              = os.path.join(os.path.abspath(os.curdir), 'data')
        self.CURR_FOLDER                        = os.path.abspath(os.curdir)
        self.CONFIG_FOLDER                      = os.path.join(self.CURR_FOLDER, 'config')
        self.CONFIG_FILEHANDLING_FILEPATH       = os.path.join(self.CONFIG_FOLDER, 'file_config.json')
        self.CONFIG_PROTOCOL_FILEPATH           = os.path.join(self.CONFIG_FOLDER, 'protocol_config.json')
        if not os.path.exists(self.CONFIG_FOLDER ): 
            try:  os.makedirs(self.CONFIG_FOLDER , exist_ok=True)
            except PermissionError: print("PERMISSION DENIED IN {}. FATAL ERROR, EXITING.".format(self.CURR_FOLDER))

        
        self.configure_protocol_frame()
        self.configure_file_handling()
        self.frame_file.place(x=500, y=10, width=600, height=150)
        self.protocol_frame.place(bordermode=tk.OUTSIDE, x=450, y=175, width=1000, height=400)

        # get starting information from arduino
        motor_info = self.get_motor_tx()
        magic = self.extract_magic_number(motor_info)
        magic = '-1'
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
            configure_button(self.button_toggle_feedback, text='Toggle Feedback\n(off)', borderwidth=2)
        else:
            self.feedback_active_flag[self.well_motor] = True
            configure_button(self.button_toggle_feedback, text='Toggle Feedback\n(on)', borderwidth=5)

    def configure_row(self, i):
        def move_camera(motor=i):
            string = "C{},{}\n".format(motor, self.well_locations[motor])

            self.well_string.set(self.well_labels[motor] + self.well_string.get()[1:])
            self.write_to_motors(string)
            self.button_move_to[self.well_motor].configure(text='Move Camera\nto Row {}'.format(self.well_labels[self.well_motor]))
            self.button_move_to[self.well_motor].configure(borderwidth=2)
            self.button_move_to[self.well_motor].configure(state='normal')
            self.well_motor = motor
            self.button_move_to[self.well_motor].configure(text='Camera Under\nRow {}'.format(self.well_labels[self.well_motor]))
            self.button_move_to[self.well_motor].configure(borderwidth=7)
            self.button_move_to[self.well_motor].configure(state='disabled')
            
            # self.frame_initialization.place(x=330, y=315-(motor*105))
            if self.motors[self.well_motor].initialized == True:
                configure_button(self.button_initialize_camera, 'Reset\nCamera')
            else:
                configure_button(self.button_initialize_camera, 'Initialize\nCamera')
            # configure_button(self.button_initialize_camera,    location=[350, 315-(motor*105), 100, 40])
            # configure_button(self.button_home_motor,           location=[350, 350-(motor*105), 100, 25])
            # configure_button(self.button_adj_down,             location=[450, 350-(motor*105), 20, 20])
            # configure_button(self.button_adj_up,               location=[330, 350-(motor*105), 20, 20])
            # configure_button(self.button_toggle_feedback,      location=[340, 400-(motor*105), 120, 40])
            # print([[350, 315-(i*105), 100, 40]])
        def retract_motor(motor=i):
            string = "X{}\n".format(motor)
            self.write_to_motors(string)
        def enable_motor(motor=i):
            string = "M{}\n".format(motor)
            self.write_to_motors(string)
            if self.motors[motor].enabled:  configure_button(self.button_start[motor],text='Stop', state='normal',borderwidth=5, bg='green')
            else:                           configure_button(self.button_start[motor],text='Start', state='normal',borderwidth=2, bg='green')
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
        configure_button(self.button_move_to[i],   "Move to Row {}".format(self.well_labels[i]), move_camera, 12, location=[0,0,145,50])
        configure_button(self.button_retract[i],   'Retract Motor', retract_motor,location=[155,50,135,40], fontsize=12)
        configure_button(self.button_start[i],     'Start', enable_motor, location=[155,0,135,40], fontsize=14)
        # configure_button(self.button_waveform[i],  'Shape', open_waveform_dialog, location=[105,60,50,25], fontsize=8)
        self.button_start[i].configure(self.enable_motor_configuration('stopped'))
        
        configure_button(self.button_camera_up[i], "+", move_camera_up, 12, [75, 70, 25, 25])
        configure_button(self.button_camera_down[i], "-", move_camera_down, 12, [25, 70, 25, 25])
        
        # ADD LABELS
        configure_label(tk.Label(self.frames[i]), "Camera Adjust: ", fontsize=8, anchor='e', location=[25, 50, -1, -1])

    def configure_global_commands(self):
        self.button_reset_camera_position   = configure_button(tk.Button(self.root), "Reset Camera\nPosition", self.reset_camera_position, 10, [100, 430, 100, 50])
        self.button_retract_all_motors      = configure_button(tk.Button(self.root), "Retract All\nMotors",    self.retract_all_motors,    10, [200, 430, 100, 50])
        self.button_stop_all_motors         = configure_button(tk.Button(self.root), "Stop All\nMotors",       self.stop_all_motors,       10, [0, 430, 100, 50])
        self.button_teardown                = configure_button(tk.Button(self.root), "Teardown\nMotors",       self.teardown_motors,       10, [200, 530, 100, 50])
        

        self.frame_initialization           = tk.Frame(self.root)
        self.button_initialize_camera       = configure_button(tk.Button(self.frame_initialization), "Initialize\nCamera",     self.initialize_camera,     10, [20, 0, 100, 40])
        self.button_home_motor              = configure_button(tk.Button(self.frame_initialization), "Home Motor",             self.home_motor,            10, [20, 40, 100, 25])
        self.button_adj_up                  = configure_button(tk.Button(self.frame_initialization), "-",                      lambda: self.send_adj(0),   10, [0, 42.5, 20, 20])
        self.button_adj_down                = configure_button(tk.Button(self.frame_initialization), "+",                      lambda: self.send_adj(1),   10, [120, 42.5, 20, 20])
        self.button_record_video            = configure_button(tk.Button(self.frame_initialization), "Record\nContinuous",     self.toggle_video_recording,10, [20, 65, 100, 40])
        self.button_record_by_ttl           = configure_button(tk.Button(self.frame_initialization), "Record\nBy TTL",         self.toggle_video_by_ttl,   10, [20, 105, 100, 40])
        # self.button_toggle_feedback         = configure_button(tk.Button(self.frame_initialization), "Toggle Feedback\n(off)", command=self.toggle_feedback, fontsize=10, location=[10, 110, 120, 40])
        
        # configure_label(tk.Label(self.frame_initialization), "Plate: ", location=[0,155,150, 25], fontsize=10, anchor='w')
        # configure_entry(tk.Entry(self.frame_initialization), self.plate_name_var, [0,180,150,25], 10)
        
        self.entry_command                  = configure_entry(tk.Entry(self.root),     textvariable=self.command, location=[25, 500, -1, -1], fontsize=10 )
        self.entry_command.bind('<Return>', func=lambda val: self.send_command())
        self.entry_command.bind

        self.frame_initialization.configure({  'background':'white', 
                                        'borderwidth':0,
                                        'relief':'groove', 
                                        'width':150, 
                                        'height':250})
        self.frame_initialization.place(x=330, y=315)

    def file_dialog(self):
        root = tk.Tk('Parent Folder for Data')
        root.withdraw()
        self.parent_directory = askdirectory(parent=root)
        root.destroy()
        if len(self.parent_directory) == 0:
            self.parent_directory = self.DEFAULT_PARENT_FOLDER
        self.parent_folder.set(self.parent_directory)
        

        self.update_filename(1)
        
    def update_filename(self, event=0, *args):
        self.warnings = {}
        self.warning_idx = 0
        
        # get subfolder name if available
        subfolder = [self.day.get(), self.experiment.get()]
        subfolder = [x for x in subfolder if x]
        self.day_and_experiment = ' - '.join(subfolder)
        
        if len(self.day.get()) == 0:        
            self.warnings[self.warning_idx] = ('day missing',1)
            self.warning_idx += 1

        if len(self.platename.get()) == 0:
            self.warnings[self.warning_idx] = ('plate missing',1)
            self.warning_idx += 1
        if len(self.well_string.get()) == 0:
            self.warnings[self.warning_idx] = ('well missing',1)
            self.warning_idx += 1


        # get different data folder if file tree is generated
        if self.generate_file_tree.get():   self.data_folder.set(os.path.join(self.parent_folder.get(), self.day_and_experiment, self.platename.get())) # subfolders for day/experiment and plate

        else:                               self.data_folder.set(os.path.join(self.parent_folder.get())) # put data into parent folder (selected by user)
            
        strings = [self.platename.get(), self.day.get(), self.well_string.get(), self.experiment.get(), self.experiment_info.get()] # all fields for the filename
        strings = [x for x in strings if x] # combine all the ones that have data in them
        self.filename.set('_'.join(strings))
        self.full_filename.set(os.path.join(self.data_folder.get(), self.filename.get()))

        self.data_folder_exists = os.path.exists(self.data_folder.get())

        if not self.data_folder_exists: # if the data folder does not exist
            if 'data folder exist' not in self.warnings.values(): # if the warning is not already present, add
                self.warnings[self.warning_idx] = ('data folder exist',0)
                self.warning_idx += 1
            pass
        
        # add all warnings to GUI
        all_txt_warnings = ''
        self.proceed_with_recording = True
        for key, value in self.warnings.items():
            if value[0] == 'data folder exist':
                all_txt_warnings += 'Data Folder does not currently exist. Creating folder tree during recording.\n'
            if value[0] == 'day missing':
                all_txt_warnings += 'ERROR: Input a value in the Day field.\n'

            if value[0] == 'plate missing':
                all_txt_warnings += 'ERROR: Input a value in the Plate field.\n'
            if value[0] == 'well missing':
                all_txt_warnings += 'ERROR: Input a value in the Well field.\n'
            
            if value[1] == 1: # if there's any warning that's an error (value[1] in dict entry is a 1, not a 0)
                self.proceed_with_recording = False # cannot proceed with recording
                
        
        self.filename_warnings.set(all_txt_warnings)

        if self.proceed_with_recording: 
            configure_button(self.button_record_video, state='active') 
            configure_button(self.button_record_by_ttl, state='active')
        else:                           
            configure_button(self.button_record_video, state='disabled')
            configure_button(self.button_record_by_ttl, state='disabled')
        
        if not self.protocol.trig.get():
            configure_button(self.button_record_by_ttl, state='disabled')
                
        W2 = font.Font(family='TkDefaultFont' , size = 8)
        length = W2.measure(self.data_folder.get())
        if length > 400: self.label_data_folder.configure(anchor = 'e')
        else:            self.label_data_folder.configure(anchor = 'w')

        json_object = json.dumps({'platename':self.platename.get(), 
                                  'day':self.day.get(), 
                                  'experiment':self.experiment.get(), 
                                  'well':self.well_string.get(), 
                                  'experiment info':self.experiment_info.get(), 
                                  'parent folder':self.parent_folder.get(),
                                  'file tree generation':self.generate_file_tree.get()})
        
        with open(self.CONFIG_FILEHANDLING_FILEPATH,'w') as f:
            f.write(json_object)
    
    def configure_file_handling(self):
        
        self.frame_file = tk.Frame(self.root)
        # define base variables for file handling
        
        self.platename          = tk.StringVar(self.frame_file, 'PLT001')
        self.day                = tk.StringVar(self.frame_file, 'd01')
        self.experiment         = tk.StringVar(self.frame_file, "")
        self.well_string        = tk.StringVar(self.frame_file, 'D4')
        self.experiment_info    = tk.StringVar(self.frame_file, '')

        self.parent_folder  = tk.StringVar(self.frame_file, self.DEFAULT_PARENT_FOLDER) # parent folder for experiment
        self.generate_file_tree = tk.IntVar(self.frame_file, 1)


        # load file configuration, if exists
        if os.path.exists(self.CONFIG_FILEHANDLING_FILEPATH):
            with open(self.CONFIG_FILEHANDLING_FILEPATH,'r') as f:
                data = json.load(f)

                self.platename.set(data['platename'])
                self.day.set(data['day'])
                self.experiment.set(data['experiment'])
                self.well_string.set(data['well'])
                self.experiment_info.set(data['experiment info'])

                self.parent_folder.set(data['parent folder'])
                self.generate_file_tree.set(data['file tree generation'])

        self.day_and_experiment = ' - '.join(self.day.get())

        # define entry for each string
        configure_label(tk.Label(self.frame_file), "Plate: ", 8,           [0,   30, 100, 20], 'w')
        configure_entry(tk.Entry(self.frame_file), self.platename,         [0,   50, 100, 25], 10)

        configure_label(tk.Label(self.frame_file), "Day: ", 8,             [100, 30, 50,  20], 'w')
        configure_entry(tk.Entry(self.frame_file), self.day,               [100, 50, 50,  25], 10)

        configure_label(tk.Label(self.frame_file), "Experiment: ", 8,      [150, 30, 100, 20], 'w')
        configure_entry(tk.Entry(self.frame_file), self.experiment,        [150, 50, 100, 25], 10)

        configure_label(tk.Label(self.frame_file), "Well: ", 8,            [245, 30, 25,  20], 'w')
        configure_entry(tk.Entry(self.frame_file), self.well_string,       [250, 50, 25,  25], 10)

        configure_label(tk.Label(self.frame_file), "Info: ", 8,            [275, 30, 100, 20], 'w')
        configure_entry(tk.Entry(self.frame_file), self.experiment_info,   [275, 50, 100, 25], 10)
        
        # folder, subfolder, and filename string variables
        self.data_folder    = tk.StringVar(self.frame_file, os.path.join(self.DEFAULT_PARENT_FOLDER, self.day.get(), self.platename.get()))
        self.filename       = tk.StringVar(self.frame_file, '_'.join((self.platename.get(), self.day.get(), self.well_string.get(), self.experiment_info.get())))
        self.full_filename  = tk.StringVar(self.frame_file, os.path.join(self.data_folder.get(), self.filename.get()))

        # parent folder selection
        configure_button(tk.Button(self.frame_file), "Project Folder", self.file_dialog, 10, [0, 0, 100, 25])
        self.label_data_folder = configure_label(tk.Label(self.frame_file), '', 8,   [100, 0, 400, 25], anchor = 'e', textvariable=self.data_folder)
        
        configure_label(tk.Label(self.frame_file), '', 10,  [0, 75, 500, 25], 'w', textvariable=self.filename)

        self.filename_warnings = tk.StringVar(self.frame_file, '')
        configure_label(tk.Label(self.frame_file), '', 10,  [0, 100, 500, 200], 'nw', textvariable=self.filename_warnings, justify=tk.LEFT)
        # self.plate_dropdown = tk.OptionMenu(self.root, self.plate_name_var, *self.available_plates, command=self.update_filename)
        # self.plate_dropdown.pack()


        # self.entry_filename = configure_entry(tk.Entry(self.file_frame), self.full_filename, [75, 0, 200, 25], fontsize=8, state=tk.DISABLED, justify=tk.RIGHT)
        # configure_label(tk.Label(self.frame_initialization), 'Well: ', location=[0, 105, 150, 25], fontsize=10,anchor='w')
        # configure_entry(tk.Entry(self.frame_file), self.plate_name_var,                      [0,25,100,25], 10, bind_function = ('<KeyRelease>',self.update_filename))
        # configure_entry(tk.Entry(self.frame_file), self.well_var,                            [100,25,25,25], 10, bind_function = ('<KeyRelease>',self.update_filename))
        # configure_entry(tk.Entry(self.frame_file), self.experiment_info_var,                 [125,25,150,25], 10, bind_function = ('<KeyRelease>',self.update_filename))

        self.check_generate_file_tree = tk.Checkbutton(self.frame_file, 
                                                       text='Generate Folder\nStructure', 
                                                       variable=self.generate_file_tree, 
                                                       onvalue=1, 
                                                       offvalue = 0, 
                                                       command = self.update_filename, 
                                                       bg='white')
        self.check_generate_file_tree.place(x=375, y=25)
        self.update_filename(1)


        self.platename.trace_add('write',self.update_filename)
        self.day.trace_add('write',self.update_filename)
        self.experiment.trace_add('write',self.update_filename)
        self.well_string.trace_add('write',self.update_filename)
        self.experiment_info.trace_add('write',self.update_filename)
        self.parent_folder.trace_add('write',self.update_filename)
        self.generate_file_tree.trace_add('write',self.update_filename)
        
        # configure_label(tk.Label(self.frame_initialization), "Plate: ", location=[0,155,150, 25], fontsize=10, anchor='w')
        # configure_entry(tk.Entry(self.frame_initialization), self.plate_name_var, [0,180,150,25], 10)
        
        self.frame_file.configure({  'background':'white', 
                                        'borderwidth':0,
                                        'relief':'groove', 
                                        })

    def toggle_video_by_ttl(self):
        self.running_processing = not self.running_processing
        # self.recording = not self.recording
        string = "OMV,{}&REMOVEPROCESSING\n".format(self.well_motor)
        self.monitor_ttl_for_recording = not self.monitor_ttl_for_recording
        configure_button(self.button_record_by_ttl, 
                              'Waiting\nfor TTL...' if self.monitor_ttl_for_recording else 'Record\nBy TTL', 
                              borderwidth = 4       if self.monitor_ttl_for_recording else 2) # configure button to 

        # print(self.data_folder_exists)
        if not os.path.exists(self.data_folder):
            try: 
                os.makedirs(self.data_folder.get(), exist_ok=True)
                self.data_folder_exists = True
            except PermissionError: 
                print("PERMISSION DENIED IN CREATING DATA FOLDER {}.".format(self.data_folder.get()))
                self.protocol_log[self.protocol_log_idx] = 'data folder error'
                self.protocol_log_idx += 1
                self.display_protocol_log()
                self.recording = False
                configure_button(self.button_record_by_ttl, 
                              'Waiting\nfor TTL...' if self.recording else 'Record\nBy TTL', 
                              borderwidth = 4 if self.recording else 2) # configure button to 
                
        if self.data_folder_exists and self.recording:
            self.protocol_log[self.protocol_log_idx] = 'data folder created'
            self.protocol_log_idx += 1
            self.display_protocol_log()

    def begin_ttl_recording(self):
        if self.monitor_ttl_for_recording:
            self.got_ttl = False # turn off flag for getting at ttl
            self.recording = True # turn on recording flag
            self.time_start_recording = time() # begin recording timer
            self.recording_time = 2.5 # seconds
            configure_button(self.button_record_by_ttl, 'Recording', borderwidth=5)
            self.time_of_recording = datetime.today().strftime("%Y%m%d_%H%M%S") # timestamp
            self.video_filename = '{}_videocapture_{}_{}.avi'.format(self.full_filename.get(), self.n_ttl, self.time_of_recording)
            self.txt_filename = "{}_video_information_{}_{}.txt".format(self.full_filename.get(), self.n_ttl, self.time_of_recording)

            if not os.path.exists(self.data_folder.get()):
                os.makedirs(self.data_folder.get())
            with open(self.txt_filename, 'w') as f:
                f.write('global time (ms),well,motor time (ms),motor position (steps),max distance (steps),cycle count,TrigMode,TRIG,post X (px),post Y (px), stretch (%), magnet X (px), magnet Y(px)\n')
            print(self.video_filename)
            self.videowriter =  cv2.VideoWriter(self.video_filename, 
                                cv2.VideoWriter_fourcc('I','4','2','0'),
                                self.frame_rate, self.image_size)

    def toggle_video_recording(self):
        self.running_processing = not self.running_processing
        self.recording = not self.recording
        string = "OMV,{}&REMOVEPROCESSING\n".format(self.well_motor)
        print(self.recording)
        configure_button(self.button_record_video, 
                                        'Recording' if self.recording else 'Record\nContinuous',
                         borderwidth =  5           if self.recording else 2)
        
        if not os.path.exists(self.data_folder.get()):
            try: 
                os.makedirs(self.data_folder.get(), exist_ok=True)
                self.data_folder_exists = True
            except PermissionError: 
                print("PERMISSION DENIED IN CREATING DATA FOLDER {}.".format(self.data_folder.get()))
                self.protocol_log[self.protocol_log_idx] = 'data folder error'
                self.protocol_log_idx += 1
                self.display_protocol_log()
                self.recording = False
                configure_button(self.button_record_video, 
                                        'Recording'         if self.recording else 'Record\nContinuous',
                                        borderwidth =  5    if self.recording else 2)
                
        if self.recording:
            self.time_of_recording = datetime.today().strftime("%Y%m%d_%H%M%S")
            self.video_filename = '{}_videocapture_{}.avi'.format(self.full_filename.get(), self.time_of_recording)
            self.txt_filename = "{}_video_information_{}.txt".format(self.full_filename.get(), self.time_of_recording)
            self.videowriter = cv2.VideoWriter(self.video_filename, 
                         cv2.VideoWriter_fourcc('I','4','2','0'),
                         self.frame_rate, self.image_size)
            with open(self.txt_filename, 'w') as f:
                f.write('global time (ms),well,motor time (ms),motor position (steps),max distance (steps),cycle count,TrigMode,TRIG,post X (px),post Y (px), stretch (%), magnet X (px), magnet Y(px)\n')
        
        else:
            if self.videowriter is not None:
                try:    self.videowriter.release()
                except: pass
                self.videowriter = None
            

        self.write_to_motors(string)
   
    def configure_protocol_frame(self):
        self.protocol_frame = tk.Frame(self.root)
        
        configure_label(tk.Label(self.protocol_frame), 'PROTOCOL', 12, [0, 0, 400, 30], anchor='c')
        
        self.protocol = Protocol(self.protocol_frame)

        if os.path.exists(self.CONFIG_PROTOCOL_FILEPATH):
            with open(self.CONFIG_PROTOCOL_FILEPATH,'r') as f:
                data = json.load(f)
                # print(data)
                self.protocol.update(steps          = data['steps'],
                                     pre_delay      = data['pre_delay'],
                                     rise           = data['rise'],
                                     hold           = data['hold'],
                                     fall           = data['fall'],
                                     post_delay     = data['post_delay'],
                                     n_cycles       = data['n_cycles'],
                                     trig           = data['trig'],
                                     assigned_to    = data['assigned_to'])
                

        self.speed_rise_var = tk.StringVar(self.protocol_frame, '{} steps/s'.format(self.motors[self.well_motor].speed_rise))
        self.speed_fall_var = tk.StringVar(self.protocol_frame, '{} steps/s'.format(self.motors[self.well_motor].speed_fall))
        
        self.t = np.cumsum([0, self.protocol.pre_delay.get(), self.protocol.rise.get(),  self.protocol.hold.get(),  self.protocol.fall.get(), self.protocol.post_delay.get()])
        self.d =           [0, 0,                             self.protocol.steps.get(), self.protocol.steps.get(), 0,                        0]
        
        self.entry_steps      = configure_entry(tk.Entry(self.protocol_frame), self.protocol.steps,         [200, 30, 50, 25])
        self.entry_pre_delay  = configure_entry(tk.Entry(self.protocol_frame), self.protocol.pre_delay,     [200, 55, 50, 25])
        self.entry_rise       = configure_entry(tk.Entry(self.protocol_frame), self.protocol.rise,          [200, 80, 50, 25])
        self.entry_hold       = configure_entry(tk.Entry(self.protocol_frame), self.protocol.hold,          [200, 105, 50, 25])
        self.entry_fall       = configure_entry(tk.Entry(self.protocol_frame), self.protocol.fall,          [200, 130, 50, 25])
        self.entry_post_delay = configure_entry(tk.Entry(self.protocol_frame), self.protocol.post_delay,    [200, 155, 50, 25])
        self.entry_n_cycles   = configure_entry(tk.Entry(self.protocol_frame), self.protocol.n_cycles,      [200, 180, 50, 25])

        configure_label(tk.Label(self.protocol_frame), 'motor distance (steps):',     10, [0, 30, 200, 25], anchor='e')
        configure_label(tk.Label(self.protocol_frame), 'pre-stretch delay (ms):',     10, [0, 55, 200, 25], anchor='e') 
        configure_label(tk.Label(self.protocol_frame), 'stretch duration (ms):',      10, [0, 80, 200, 25], anchor='e')
        configure_label(tk.Label(self.protocol_frame), 'hold duration (ms):',         10, [0, 105, 200, 25], anchor='e')
        configure_label(tk.Label(self.protocol_frame), '>=100 ms',                     10, [250, 105, 200, 25], anchor='w')
        configure_label(tk.Label(self.protocol_frame), 'un-stretch duration (ms):',   10, [0, 130, 200, 25], anchor='e')
        configure_label(tk.Label(self.protocol_frame), 'rest duration (ms):',         10, [0, 155, 200, 25], anchor='e')
        configure_label(tk.Label(self.protocol_frame), '>=100 ms',                     10, [250, 155, 200, 25], anchor='w')
        configure_label(tk.Label(self.protocol_frame), 'N cycles (-1 for inf):',      10, [0, 180, 200, 25], anchor='e')
        
        tk.Checkbutton(self.protocol_frame, 
                       text         = 'External\nTrigger', 
                       variable     = self.protocol.trig, 
                       onvalue      = 1, 
                       offvalue     = 0, 
                       bg           = 'white').place(x=260, y=180)
        
        
        
        configure_label(tk.Label(self.protocol_frame), "Assign to:", 11, [0, 250, 125, 25], 'e' )
        self.protocol_buttons = [tk.Button(self.protocol_frame), tk.Button(self.protocol_frame), tk.Button(self.protocol_frame), tk.Button(self.protocol_frame)]
        self.protocol_assigned_to_motors = [              False,                          False,                          False,                          False]
        
        # self.button_trig_toggle = configure_button(tk.Button(self.protocol_frame), "External\ntrigger", toggleExtTrig, 10, [260, 180, 60, 40])
        # self.button_save_protocol  = configure_button(tk.Button(self.protocol_frame), 'Save', self.save_protocol, 12, [250, 220, 60, 25])

        
        for i in range(4):
            self.protocol_buttons[i] = tk.Button(self.protocol_frame)
            def send_protocol(motor=i):
                
                temp = int(self.protocol.n_cycles.get())

                string = "PROTOCOL{well},{distance},{pre_stretch_delay},{stretch_duration},{hold_duration},{fall_duration},{rest_duration},{n_cycles},{trig}\n".format(
                    well=motor,
                    distance =              self.protocol.steps.get(),     
                    pre_stretch_delay =     self.protocol.pre_delay.get(),
                    stretch_duration =      self.protocol.rise.get()      ,
                    hold_duration =         self.protocol.hold.get()      ,
                    fall_duration =         self.protocol.fall.get()      ,
                    rest_duration =         self.protocol.post_delay.get(),
                    n_cycles                = temp,
                    trig =                  self.protocol.trig.get() )
                    
                if self.passed_check_protocol:
                    self.protocol.assigned_to[motor] = not self.protocol.assigned_to[motor]
                    self.update_motor_assignment()
                    if 'send protocol' not in self.protocol_log.values():
                        self.protocol_log[self.protocol_log_idx] = 'send protocol'
                        self.protocol_log_idx += 1
                    self.display_protocol_log()
                    if self.protocol.assigned_to[motor]: self.write_to_motors(string)

                    # self.motors[motor].assigned_protocol = not self.motors[motor].assigned_protocol
                    # if self.motors[motor].assigned_protocol:
                    #     self.protocol_buttons[motor].configure(borderwidth=2)
                    #     self.protocol.assigned_to[motor] = False


                    # else:
                    #     self.protocol_buttons[motor].configure(borderwidth=5)
                    #     self.protocol.assigned_to[motor] = True
                    #     self.write_to_motors(string)

                
                    
                else:
                    if 'send protocol error' not in self.protocol_log.values():
                        self.protocol_log[self.protocol_log_idx] = 'send protocol error'
                        self.protocol_log_idx += 1
                        self.display_protocol_log()
                    pass
                pass
            
            configure_button(self.protocol_buttons[i],self.well_labels[i], send_protocol, 12, [200-(i*25), 250, 25, 25])

        self.button_begin_protocols = configure_button(tk.Button(self.protocol_frame), "Start Protocol", self.begin_protocol, 12, [100, 280, 150, 25])

        self.button_save_protocol   = configure_button(tk.Button(self.protocol_frame),"Save", self.save_protocol, 12, [175, 220, 50, 25])
        self.button_load_protocol   = configure_button(tk.Button(self.protocol_frame),"Load", self.load_protocol, 12, [125, 220, 50, 25])
        
        
        self.label_protocol_begin = configure_label(tk.Label(self.protocol_frame), "", 10, [250, 280, 150, 70], anchor='nw')
        
        self.protocol_display = configure_label(tk.Label(self.protocol_frame), "", 12, [350, 200, 500, 300], anchor='nw', justify=tk.LEFT)

        # add plot canvas
        # self.fig, ax = plt.subplots(1,1)
        # self.fig.set_size_inches(1, 0.5)
        
        self.fig = Figure(figsize=(4,2), dpi=100)
        self.fig.subplots_adjust(left=0, bottom=0.15)
        self.ax = self.fig.gca()
        self.protocol_lines = self.ax.plot(self.t, self.d)
        self.ax.spines['top'].set_visible(False)
        self.ax.spines['right'].set_visible(False)
        self.ax.set_xticks(np.unique(self.t))
        self.ax.set_yticks(np.unique(self.d))
        self.ax.set_xticklabels([])

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.protocol_frame)  # A tk.DrawingArea.
        self.canvas.draw()
        

        self.canvas.get_tk_widget().place(x=350, y=0, width=500, height=200)
        # self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.X, expand=True)
        

        self.protocol_frame.configure({'background':'white', 
                                        'borderwidth':0,
                                        'relief':'groove', 
                                        'width':1000, 
                                        'height':500})
        
        
        self.protocol.pre_delay.        trace_add('write',self.change_protocol)
        self.protocol.rise.             trace_add('write',self.change_protocol)
        self.protocol.hold.             trace_add('write',self.change_protocol)
        self.protocol.fall.             trace_add('write',self.change_protocol)
        self.protocol.post_delay.       trace_add('write',self.change_protocol)
        self.protocol.steps.            trace_add('write',self.change_protocol)
        self.protocol.n_cycles.         trace_add('write',self.change_protocol)
        self.protocol.trig.             trace_add('write',self.change_protocol)

        self.change_protocol()

    def save_protocol(self, *args):
        if self.passed_check_protocol:
            self.protocol_log = {}
            self.protocol_log_idx = 0
            self.save_protocol_filename = asksaveasfilename(defaultextension='json')
            if len(self.save_protocol_filename) > 0:
                json_object = json.dumps({
                        'steps':        self.protocol.steps        .get(),
                        'pre_delay':    self.protocol.pre_delay    .get(),
                        'rise':         self.protocol.rise         .get(),
                        'hold':         self.protocol.hold         .get(),
                        'fall':         self.protocol.fall         .get(),
                        'post_delay':   self.protocol.post_delay   .get(),
                        'n_cycles':     self.protocol.n_cycles     .get(),
                        'trig':         self.protocol.trig         .get(),
                        'assigned_to':  self.protocol.assigned_to
                        })

                with open(self.save_protocol_filename,'w') as f:
                    f.write(json_object)

            self.protocol_log[self.protocol_log_idx] = 'save protocol'
            self.protocol_log_idx += 1

            self.display_protocol_log()
        else:
            if 'save protocol error' not in self.protocol_log.values():
                self.protocol_log[self.protocol_log_idx] = 'save protocol error'
                self.protocol_log_idx += 1
                self.display_protocol_log()

    def load_protocol(self, *args):

        self.open_protocol_filename = askopenfilename(filetypes = [('JSON','*.json')])
        self.protocol_log = {}
        self.protocol_log_idx = 0
        if os.path.exists(self.open_protocol_filename):
            with open(self.open_protocol_filename,'r') as f:
                data = json.load(f)
                if set(['steps','pre_delay','rise','hold','fall','post_delay','n_cycles','trig','assigned_to']).issubset(set(data.keys())):
                    # print(data)
                    self.protocol.update(steps          = data['steps'],
                                        pre_delay      = data['pre_delay'],
                                        rise           = data['rise'],
                                        hold           = data['hold'],
                                        fall           = data['fall'],
                                        post_delay     = data['post_delay'],
                                        n_cycles       = data['n_cycles'],
                                        trig           = data['trig'],
                                        assigned_to    = data['assigned_to'])
                    
                    self.protocol_log[self.protocol_log_idx] = 'load protocol'
                    self.protocol_log_idx += 1
                    self.display_protocol_log()
                else:
                    self.protocol_log[self.protocol_log_idx] = 'load protocol error'
                    self.protocol_log_idx += 1
                    self.display_protocol_log()

        pass

    def change_protocol(self, *args):

        self.protocol.assigned_to = [False] * 4
        self.update_motor_assignment(self)
        self.protocol_log = {}
        self.protocol_log_idx = 0
        self.verify_protocol(self)
        # self.update_plot(self)

        configure_button(self.button_record_by_ttl, state = 'active' if self.protocol.trig.get() else 'disabled')
        
        self.display_protocol_log(self)

    def update_motor_assignment(self, *args):
        for i in range(4):
            self.protocol_buttons[i].configure(borderwidth = 5 if self.protocol.assigned_to[i] else 2)

    def verify_protocol(self, *args):
        try:
            pre_delay       = int(self.protocol.pre_delay   .get())
            rise            = int(self.protocol.rise        .get())
            hold            = int(self.protocol.hold        .get())
            fall            = int(self.protocol.fall        .get())
            post_delay      = int(self.protocol.post_delay  .get())
            steps           = int(self.protocol.steps       .get())
            n_cycles        = int(self.protocol.n_cycles    .get())
            trig            = int(self.protocol.trig        .get())
            self.passed_check_protocol = True
            
        except:
            self.passed_check_protocol = False
            self.protocol_log[self.protocol_log_idx] = 'failed to get numbers'
            self.protocol_log_idx += 1
            return
        
        if steps > 2000:
            self.protocol_log[self.protocol_log_idx] = 'steps too large'
            self.passed_check_protocol = False
            self.protocol_log_idx += 1

        if hold < 100 or pre_delay < 100 or post_delay < 100:
            self.protocol_log[self.protocol_log_idx] = 'delay too small'
            self.passed_check_protocol = False
            self.protocol_log_idx += 1
        
        if n_cycles == 0 or n_cycles < -1:
            self.protocol_log[self.protocol_log_idx] = 'n cycles not valid'
            self.passed_check_protocol = False
            self.protocol_log_idx += 1
        
        speed_rise = steps/(rise/1000)
        speed_fall = steps/(fall/1000)

        self.speed_rise_var.set("{} steps/s".format(int(speed_rise)))
        self.speed_fall_var.set("{} steps/s".format(int(speed_fall)))

        if speed_rise > 2000 or speed_fall > 2000:
            self.protocol_log[self.protocol_log_idx] = 'speed too large'
            self.passed_check_protocol = False
            self.protocol_log_idx += 1
        
        if self.passed_check_protocol:
            json_object = json.dumps({
                'steps':        self.protocol.steps        .get(),
                'pre_delay':    self.protocol.pre_delay    .get(),
                'rise':         self.protocol.rise         .get(),
                'hold':         self.protocol.hold         .get(),
                'fall':         self.protocol.fall         .get(),
                'post_delay':   self.protocol.post_delay   .get(),
                'n_cycles':     self.protocol.n_cycles     .get(),
                'trig':         self.protocol.trig         .get(),
                'assigned_to':  self.protocol.assigned_to
                })

            with open(self.CONFIG_PROTOCOL_FILEPATH,'w') as f:
                f.write(json_object)
            
    def update_protocol_display(self):
        if self.message.count('\n') > 8:
            self.message = self.message[(self.message.index('\n')+1):]
        # print(self.message)
        self.protocol_display.configure(text=self.message)
        pass

    def display_protocol_log(self, *args):
        self.message = ''
        if 'load protocol error' in self.protocol_log.values():
            self.message += 'Cannot load protocol in {}.\n'.format(Path(self.open_protocol_filename).name)
        
        if self.passed_check_protocol == False:
            self.message += 'Invalid Protocol.\n'

            for value in self.protocol_log.values():
                # print(value)
                if value == 'Failed to get numbers':
                    self.message += ' - Invalid Entries.\n'
                    break
                if value == 'steps too large':
                    self.message += ' - Distance (steps) too large.  Limit is 2000 steps.\n'
                if value == 'speed too large':
                    self.message += ' - Speed too large.  Limit is 2000 steps/s.\n'
                    # print(self.message)
                if value == 'delay too small':
                    self.message += ' - Pre-delay, post-delay, or hold duration must be >100 ms.\n'
                if value == 'n cycles not valid':
                    self.message += ' - N cycles must be >= 1 or -1 (for inf).\n'
                
                if value == 'send protocol error':
                    self.message += ' - Cannot send invalid protocol to motor.\n'
                if value == 'save protocol error':
                    self.message += ' - Cannot save invalid protocol.\n'

        if self.passed_check_protocol:
            if 'load protocol' in self.protocol_log.values():
                self.message += 'Protocol Loaded from {}.\n'.format(Path(self.open_protocol_filename).name)
            else:
                self.message += 'Valid Protocol.\n'
            self.message += ' - Total duration is {} ms.\n'.format(self.t[-1])
            self.message += ' - Stretch duration is {} ms.\n'.format(self.t[4] - self.t[1])

            if self.protocol.trig.get():
                if self.protocol.n_cycles.get() == 1:       self.message += ' - External Trigger. One stretch per trig.\n'
                else:                                       self.message += ' - External Trigger. WARNING: More than one stretch per trig.\n'
            else:
                if self.protocol.n_cycles.get() == 1:       self.message += ' - Open Loop. One cycle.\n'
                else:
                    if self.protocol.n_cycles.get() == -1:  self.message += ' - Open Loop. Inf cycles.\n'
                    else:                                   self.message += ' - Open Loop. {} cycles.\n'.format(self.protocol.n_cycles.get())

            for value in self.protocol_log.values():
                if value == 'send protocol':
                    self.message += ' - Active Wells: ' + ', '.join(np.flip(np.array(self.well_labels)[self.protocol.assigned_to]))
                if value == 'save protocol':
                    self.message += ' - Saved protocol to {}.\n'.format(Path(self.save_protocol_filename).name)
                if value == 'data folder error':
                    self.message += ' ERROR - cannot create data folder.\n'
                if value == 'data folder created':
                    self.message += ' - Saving data to {}\n'.format('../' + '/'.join(self.data_folder.get().split('\\')[-2:]))

        self.update_protocol_display()
        pass
    
    def update_plot(self,*args):
        try: self.t = np.cumsum([0, self.protocol.pre_delay.get(), self.protocol.rise.get(), self.protocol.hold.get(), self.protocol.fall.get(), self.protocol.post_delay.get()])
        except: pass
        
        try: self.d = [0, 0, self.protocol.steps.get(), self.protocol.steps.get(), 0, 0]
        except: pass
        # self.fig.gca().clear()
        print(self.protocol_lines)
        self.protocol_lines[0].xdata = self.t
        self.protocol_lines[0].ydata = self.d
        
        ticks = np.unique(self.t)
        self.ax.set_xticks(ticks)
        labels = ['']*len(ticks)
        labels[-1] = str(self.t[-1])
        self.fig.gca().set_xticklabels(labels, fontsize=10)
        self.fig.gca().set_yticks(np.unique(self.d))
        self.fig.gca().set_yticklabels(np.unique(self.d), fontsize=10)
        # print(self.entry_steps.get())
        
        self.canvas.draw()
        if self.passed_check_protocol:
            RotationAwareAnnotation(s=self.speed_rise_var.get(), xy=(self.t[1], 0),                         p = (self.t[2], self.protocol.steps.get()), ax = self.fig.gca(), fontsize=10)
            RotationAwareAnnotation(s=self.speed_fall_var.get(), xy=(self.t[3], self.protocol.steps.get()), p = (self.t[4], 0),                         ax = self.fig.gca(), fontsize=10)

        if self.motors[self.well_motor].homed:
            pass
            self.fig.gca().axvline(self.motors[self.well_motor].t, linewidth=3, color='r')
        self.canvas.draw()
    
    def begin_protocol(self):
        string = "E{}{}{}{}\n".format(int(self.protocol.assigned_to[0]),
                                      int(self.protocol.assigned_to[1]),
                                      int(self.protocol.assigned_to[2]),
                                      int(self.protocol.assigned_to[3]))
        self.write_to_motors(string)

        self.root.after(1)

        enabled = [0,0,0,0]
        for i in range(4):
            enabled[i] = self.motors[i].enabled
        
        # print(enabled)
        # print(self.protocol.assigned_to)
        
        if enabled == self.protocol.assigned_to:
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
    
    def update(self):
        self.t1 = time() - self.t0
        # print(self.frame_rate)
        # print(round(time()-self.t0, 1))        
        # if self.waveform_root.winfo_viewable() == 1:
        #     self.plot_waveform()
        # print(time() - self.t0)

        motor_info = self.get_motor_tx()
        # print(motor_info)
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

            if len(self.last_framerates) > 30:
                self.last_framerates.pop(0)
            self.last_framerates.append(self.frame_rate)
            printfunc("{}, trig: {}, stretch: {:3.2f}, frame_rate: {:3.1f}".format(temp, self.ttl_received, self.tracking_stretch, np.mean(self.last_framerates)), end='\r')
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

        if self.motors[self.well_motor].enabled:
            # self.update_plot()
            pass
            
        # pygame.draw.rect(screen, (255,0,0), rectangle)
             
            # pygame.draw.rect(screen, (255,255,255), pygame.rect.Rect(min(x1,x2), min(y1,y2), abs(x1-x2), abs(y1-y2)))
        
        t2 = time() - self.t0
        if t2 > self.t1:
            self.frame_rate = 1/(t2-self.t1)

        if root.conn != None:
            self.root.after(1, self.update)
        else:
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
            # print(' ')
            # print(tracking_information)
            # print(' ')
            if len(tracking_information) < 11:
                return -1
            
        
            self.tracking_t = int(tracking_information[0])
            self.tracking_well = int(tracking_information[1])
            self.tracking_motor_homed = bool(int(tracking_information[2]))
            self.tracking_recieved_motor_init_handshake = bool(int(tracking_information[3]))
            self.tracking_stretch = float(tracking_information[4])
            self.tracking_found_max = bool(int(tracking_information[5]))
            self.tracking_magnet_centroid = (int(tracking_information[6]), int(tracking_information[7]) )
            self.tracking_post_centroid = (int(tracking_information[8]), int(tracking_information[9]) )
            
            # self.tracking_maxes = json.loads(tracking_information[6])
            self.magic = tracking_information[10]
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
                self.got_ttl = True
                if self.monitor_ttl_for_recording:
                    self.begin_ttl_recording()
                self.n_ttl += 1
            

            self.ttl_received = int(ttl_recieved)
            self.ttl_mode = int(ttl_mode)
            
            # printfunc("{}: {}".format(t, self.comm_time))
            # print("{}, {}, {}, {}, {}".format(temp, t, tracking_information, motor_information, stage_information))
            # temp, t, camera_information, motor_information[0], motor_information[1], motor_information[2], motor_information[3], stage_information  = tuple(string.split(';'))
            return 0
        return -1

    def extract_magic_number(self, string):
        if self.conn != None:
            magic = string.split(';')[0]
            try: magic = int(magic)
            except ValueError: magic = -99
            return magic
        else:
            magic = -99
    
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
            string = 'OMV,{}&INIT#\n'.format(self.well_motor)
            printfunc('Initializing camera.')
            configure_button(self.button_initialize_camera, 'Reset\nCamera')
        else:
            string = "OMV,{}&DEINIT#\n".format(self.well_motor)
            printfunc('Uninitializing camera.')
            configure_button(self.button_initialize_camera, 'Initialize\nCamera')
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
                # if event.button == 1:
                #     self.making_post_or_magnet = "post"
                #     self.make_a_rect = True
                #     if self.rect_making == False:
                #         self.x1, self.y1 = event.pos
                #         self.x2, self.y2 = event.pos
                #         self.rect_making = True
                if event.button == 3:
                    self.making_post_or_magnet = "post"
                    self.make_a_rect = True
                    if self.rect_making == False:
                        self.x1, self.y1 = event.pos
                        self.x2, self.y2 = event.pos
                        self.rect_making = True
                    
            elif event.type == pygame.MOUSEBUTTONUP:
                # if event.button == 1:
                #     self.rect_making = False
                #     self.make_a_rect = False
                #     if self.x1 != self.x2 and self.y1 != self.y2:
                #         self.write_to_motors("OMV,{}&POSTROI&{}&{}&{}&{}#\n".format(self.camera_well, min(self.x1, self.x2), max(self.x1, self.x2), min(self.y1, self.y2), max(self.y1, self.y2)))
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
        frame_buffer = None
        while frame_buffer == None:
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
            
            if self.recording: # if the recording flag is True
                self.videowriter.write(frame) # write frame to video
                with open(self.txt_filename, 'a') as f:

                    f.write('{t},{well},{motor_t},{motor_p},{motor_d},{cycle_count},{ttl_mode},{ttl_received},{n_ttl},{post_x},{post_y},{stretch:3.2f},{magnet_x},{magnet_y}\n'.format(
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
                        post_y = self.post_manual[1],
                        stretch = self.tracking_stretch,
                        magnet_x = self.tracking_magnet_centroid[0],
                        magnet_y = self.tracking_magnet_centroid[1]
                        ))
                    
                if self.monitor_ttl_for_recording: # if the recording mode is by ttl, and recording is active
                    if time() > self.time_of_recording + self.recording_time: # if the time of the recording exceeds recording time
                        self.recording = False # turn off recording
                        configure_button(self.button_record_by_ttl, 'Waiting\nfor TTL...', borderwidth=3) # configure button to show waiting
            
            
            
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

if __name__ == "__main__":
    root = CS3D()

    root.run_update()
    root.mainloop()