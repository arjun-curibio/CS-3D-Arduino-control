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

class Motor:
    def __init__(self, ID):
        self.ID = ID
        self.t = 0
        self.p = 0
        self.d = 0
        self.f = 0


        self.enabled = False
        self.homed = False
        self.initialized = False
        self.homing_stage = 0

class CS3D:
    def __init__(self):
        root = tk.Tk()
        root.geometry("800x500")
        root.configure({"background":'white'})
        root.resizable(0,0)
        root.protocol("WM_DELETE_WINDOW", self.destroy)
        
        
        self.ports = EstablishConnections()
        self.ports.connect_to_camera()
        self.conn = self.ports.connect_to_motors()

        pygame.init()
        self.screen = pygame.display.set_mode((640,480), pygame.DOUBLEBUF)
        self.got_image = False
        self.clock = pygame.time.Clock()
        self.screen_font = pygame.font.SysFont("monospace", 33)
        

        self.well_labels    = ['D',                         'C',                        'B',                        'A']
        self.well_locations = [0,                           4000,                       7900,                       11800]
        self.motors         = [Motor(0),                    Motor(1),                   Motor(2),                   Motor(3)]
        self.frames         = [tk.Frame(root),              tk.Frame(root),             tk.Frame(root),             tk.Frame(root)]
        self.button_move_to = [tk.Button(self.frames[0]),   tk.Button(self.frames[1]),  tk.Button(self.frames[2]),  tk.Button(self.frames[3])]
        self.button_retract = [tk.Button(self.frames[0]),   tk.Button(self.frames[1]),  tk.Button(self.frames[2]),  tk.Button(self.frames[3])]
        self.button_start   = [tk.Button(self.frames[0]),   tk.Button(self.frames[1]),  tk.Button(self.frames[2]),  tk.Button(self.frames[3])]
        self.button_waveform= [tk.Button(self.frames[0]),   tk.Button(self.frames[1]),  tk.Button(self.frames[2]),  tk.Button(self.frames[3])]
        self.motor_display  = [tk.Label(self.frames[0]),    tk.Label(self.frames[1]),   tk.Label(self.frames[2]),   tk.Label(self.frames[3])]
        
        self.well = 0
        self.root = root

        for i in range(4):
            def move_camera(motor=i):
                string = "C{},{}\n".format(motor, self.well_locations[motor])
                self.write_to_motors(string)
            def retract_motor(motor=i):
                string = "X{}\n".format(motor)
                self.write_to_motors(string)
            def enable_motor(motor=i):
                string = "M{}\n".format(motor)

            # PLACE FRAME FOR MOTOR
            self.frames[i].configure({  'background':'white', 
                                        'borderwidth':2,
                                        'relief':'groove', 
                                        'width':500, 
                                        'height':100})
            self.frames[i].place(bordermode=tk.OUTSIDE, x=0, y=315-(i*105))

            # ADD BUTTONS
            self.configure_button(self.button_move_to[i],   "Move to Row {}".format(self.well_labels[i]), move_camera, 12, location=[0,0,125,40])
            self.configure_button(self.button_retract[i],   'Retract\nMotor', retract_motor,location=[50,60,50,25], fontsize=8)
            self.configure_button(self.button_start[i],     'Start', enable_motor, location=[0,60,50,25], fontsize=8)
            self.button_start[i].configure({'state':'disabled', 'bg':'gray'})
    
    
    def configure_button(self, button, text, command, fontsize, location, **kwargs):
        button.configure({'text':text, 'background':'white', 'command':command, 'font':font.Font(size=fontsize), 'borderwidth':2, 'relief':'groove'})
        button.place(x=location[0], y=location[1], width=location[2], height=location[3])
        return button
    
    def update(self):
        if self.ports.connected_to_camera:
            self.image = self.get_frame_buffer()
            tx_camera = self.get_camera_tx()
            if self.got_image: # got an image in the frame buffer
                self.got_image = False

            if not pyopenmv.script_running(): # stopped running the script
                self.screen.blit(self.font.render("NOT CONNECTED TO CAMERA!", 1, (255, 0, 0)), (0, 0))
                pygame.display.flip()

            if len(tx_camera) > 0: # frame buffer had tx over serial port
                self.screen.blit(self.font.render("TX: {}".format(tx_camera), 1, (255, 0, 0)), (0, 0))

            pygame.display.flip() # update screen

            self.do_pygame_events()
        

        pass

    def do_pygame_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT():
                self.destroy()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    self.post_manual = event.pos
                    self.write_to_motors("POSTMANUAL{},{}\n".format(self.post_manual[0], self.post_manual[1]))
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c: # C
                    pygame.image.save(self.image, "{}_capture_{}.png".format(datetime.today().strftime("%Y%m%d_%H%M%S"), self.well_labels[self.well]))
                

    def write_to_motors(self, string):
        if self.conn is not None:
            self.conn.write(string.encode())
        printfunc("{}: {}".format(self.clock.get_rawtime(), string))
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
            # self.screen.blit(self.font.render("FPS %.2f"%(fps), 1, (255, 0, 0)), (0, 0))            
            return image
        else:
            return None

    def get_camera_tx(self):
        # a = pyopenmv.tx_buf_len()
        out = pyopenmv.tx_buf(pyopenmv.tx_buf_len()).decode()[:-2]
        return out

    def destroy(self):
        self.root.destroy()
        sys.exit()

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
        [print("{}: {} ({})".format(port, desc, hwid)) for port, desc, hwid in self.ports]
        
    
    def connect_to_camera(self):
        for port, desc, hwid in comports():
            if 'OpenMV' in port or 'OpenMV' in desc or 'OpenMV' in hwid:
                try:
                    pyopenmv.init(port, baudrate=921600, timeout=0.010)
                    pyopenmv.set_timeout(1*2) # SD Cards can cause big hicups.
                    pyopenmv.stop_script() # stop any script thats working
                    pyopenmv.enable_fb(True) # enable frame buffer (NEEDS TO BE ENABLED TO GRAB FROM FRAME BUFFER)

                    try:
                        with open('D:/main.py', 'r') as f:
                            script=f.read()
                    except:
                        script = ""

                    pyopenmv.exec_script(script) # Execute the script that's located in D:/main.py
                    self.connected_to_camera = True
                    print("Connected to camera.")
                except:
                    self.connected_to_camera=False
                    print("Failed to connect to camera.")
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
                    self.conn = Serial(port, baudrate=9600, timeout=1)
                    self.connected_to_motors = True
                    print('Connected to motors. ')
                    continue
                except:
                    self.conn = None
                    self.connected_to_motors = False
                    print('Tried to connect but failed.  Trying another or exiting.')
                    continue
        return self.conn
    
    def close(self):
        self.conn.close()

if __name__ == "__main__":
    root = CS3D()

    root.mainloop()