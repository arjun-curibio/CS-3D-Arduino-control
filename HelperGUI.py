from pydoc import Helper
from serial import Serial
import tkinter as tk
import threading
from RangeSlider.RangeSlider import RangeSliderH
import pygame, pyopenmv
from serial.tools.list_ports import comports
from datetime import datetime
import sys

class EstablishConnections:
    def __init__(self):
        self.ArduinoConn = None
        self.ArduinoConnected = True
        self.OMVConnected=False


        k=0
        for port, desc, hwid in comports():
            print("{}: {} / {} ".format(port, desc, hwid))
            if 'Bluetooth' in port:
                k = k+1
                continue
            elif 'OpenMV' in port or 'OpenMV' in desc or 'OpenMV' in hwid:
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
                    self.OMVConnected = True
                    print("Connected to OpenMV")
                except:
                    self.OMVConnected=False
                    print("Failed to connect to OpenMV")

                continue
            else:  
                try:
                    self.conn = Serial(port, baudrate=9600, timeout=1)
                    self.ArduinoConnected = True
                    print('Connected to Arduino! ')
                    continue
                except:
                    print('Tried to connect but failed.  Trying another or exiting.')
                    continue
        
        if type(self.conn) == list:
            print('No Serial port found.')
            self.ArduinoConnected = False
    
    def close(self):
        self.conn.close()

class HELPER_GUI:
    def __init__(self, root, ser=None):
        self.root = root
        root.title('HELPER')
        

        self.ser = ser
        if ser != None:
            self.conn = ser.conn

        
        pygame.init()
        self.screen = pygame.display.set_mode((640,480), pygame.DOUBLEBUF)
        self.gotImage = False
        self.Clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("monospace", 33)
        self.imageMessage = None
        
        # self.conn = conn

        self.magnet_thresh_low = tk.IntVar(self.root, 0)
        self.magnet_thresh_high = tk.IntVar(self.root, 50)
        self.post_thresh_low = tk.IntVar(self.root, 0)
        self.post_thresh_high = tk.IntVar(self.root, 50)
        self.magnet_area_low = tk.IntVar(self.root, 0)
        self.magnet_area_high = tk.IntVar(self.root, 50)
        self.post_area_low = tk.IntVar(self.root, 0)
        self.post_area_high = tk.IntVar(self.root, 50)
        self.magnet_extent = tk.DoubleVar(self.root, 0.5)
        self.magnet_aspectratio_low = tk.DoubleVar(self.root, 0.9)
        self.magnet_aspectratio_high = tk.DoubleVar(self.root, 10)
        
        self.post_circularity = tk.DoubleVar(self.root, 0.9)
        
        self.rs_magnet_thresh = RangeSliderH(self.root, [self.magnet_thresh_low, self.magnet_thresh_high], min_val = 0, max_val = 155, padX=20, digit_precision='1.0f')
        self.rs_post_thresh = RangeSliderH(self.root, [self.post_thresh_low, self.post_thresh_high], min_val = 0, max_val = 50,  padX=20, digit_precision='1.0f')
        self.rs_magnet_area = RangeSliderH(self.root, [self.magnet_area_low, self.magnet_area_high], min_val = 500, max_val = 5000,  padX=40, digit_precision='4.0f')
        self.rs_post_area = RangeSliderH(self.root, [self.post_area_low, self.post_area_high], min_val = 0, max_val = 25000,  padX=40, digit_precision='4.0f')
        
        self.s_magnet_extent        = tk.Scale(self.root, from_=0, to=1,    variable=self.magnet_extent,        resolution=0.1, orient='horizontal')
        
        self.rs_magnet_aspectratio   = RangeSliderH(self.root, [self.magnet_aspectratio_low, self.magnet_aspectratio_high],  min_val = 0, max_val = 10, padX=20, digit_precision='0.05f')

        self.s_post_circularity     = tk.Scale(self.root, from_=0, to=1,  variable=self.post_circularity,     resolution=0.05, orient='horizontal')
        
        self.rs_magnet_thresh.pack()
        self.rs_post_thresh.pack()
        self.rs_magnet_area.pack()
        self.rs_post_area.pack()
        self.s_magnet_extent.pack()
        self.rs_magnet_aspectratio.pack()
        self.s_post_circularity.pack()
        
        self.conn.write('HELPERTOGGLE\n'.encode())

        def sendToggle():
            self.conn.write("HELPERTOGGLE\n".encode())
            print("HELPERTOGGLE\n")
        self.ToggleButton = tk.Button(self.root, text='Toggle Helper', command=sendToggle)
        self.ToggleButton.pack()

        def sendPostMask():
            self.conn.write("HELPERMASK,POST\n".encode())
        self.PostMaskButton = tk.Button(self.root, text='Mask for Post', command=sendPostMask)
        self.PostMaskButton.pack()

        def sendMagnetMask():
            self.conn.write("HELPERMASK,MAGNET\n".encode())
        self.MagnetMaskButton = tk.Button(self.root, text='Mask for Magnet', command=sendMagnetMask)
        self.MagnetMaskButton.pack()

        def updateParameters():
            self.conn.write('PAR{},{},{},{},{},{},{},{},{},{},{},{}\n'.format(
            self.magnet_thresh_low.get(),
            self.magnet_thresh_high.get(),
            self.post_thresh_low.get(),
            self.post_thresh_high.get(),
            self.magnet_area_low.get(),
            self.magnet_area_high.get(),
            self.post_area_low.get(),
            self.post_area_high.get(),
            self.magnet_extent.get()*100,
            self.magnet_aspectratio_low.get()*100,
            self.magnet_aspectratio_high.get()*100,
            self.post_circularity.get()*100).encode())
        self.UpdateParametersButton = tk.Button(self.root, text='Update Algorithm w/\nAbove Parameters', command=updateParameters)
        self.UpdateParametersButton.pack()
        

    def update(self):
        self.conn.write('HELPER{},{},{},{},{},{},{},{},{},{},{},{}\n'.format(
            self.magnet_thresh_low.get(),
            self.magnet_thresh_high.get(),
            self.post_thresh_low.get(),
            self.post_thresh_high.get(),
            self.magnet_area_low.get(),
            self.magnet_area_high.get(),
            self.post_area_low.get(),
            self.post_area_high.get(),
            self.magnet_extent.get()*100,
            self.magnet_aspectratio_low.get()*100,
            self.magnet_aspectratio_high.get()*100,
            self.post_circularity.get()*100
            ).encode())
        # print('HELPER{},{},{},{},{},{},{},{},{},{},{},{}\n'.format(
        #     self.magnet_thresh_low.get(),
        #     self.magnet_thresh_high.get(),
        #     self.post_thresh_low.get(),
        #     self.post_thresh_high.get(),
        #     self.magnet_area_low.get(),
        #     self.magnet_area_high.get(),
        #     self.post_area_low.get(),
        #     self.post_area_high.get(),
        #     self.magnet_extent.get()*100,
        #     self.magnet_aspectratio_low.get()*100,
        #     self.magnet_aspectratio_high.get()*100,
        #     self.post_circularity.get()*100
        #     ))
        print(self.conn.readline())
        self.conn.flush()

        
        if self.ser.OMVConnected==True:
            image = self.getFrameBuffer()
            val = self.getOpenMVSerial()
            # if val != "":
            #     print(self.getOpenMVSerial())
        
        # update display
        if self.gotImage == True:
            pygame.display.flip()
            self.gotImage = False
        # print(pyopenmv.script_running())
        


        for event in pygame.event.get():
            # print("{}: {}".format(e,event))
            # e+=1
            if event.type == pygame.QUIT:
                self.destroy()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    self.RigidPosition = event.pos
                    self.writeToSerial("POSTMANUAL{},{}\n".format(self.RigidPosition[0], self.RigidPosition[1]))

                print("{}, {}, {}".format(event.pos, event.button, event.touch))
            elif event.type == pygame.KEYDOWN:
                # if event.key == pygame.K_ESCAPE:
                #     self.destroy()
                if event.key == pygame.K_c:
                    pygame.image.save(image, "{}_capture_{}.png".format(datetime.today().strftime('%Y%m%d_%H%M%S'),self.wellLabels[int(self.well)]))
        
        self.root.after(500, self.update)
    
    def run_update(self):
        self.thr = threading.Thread(target=self.update())
        self.thr.start()

    def mainloop(self):
        self.root.mainloop()
    


    def getFrameBuffer(self):
        fb = pyopenmv.fb_dump()
        if fb != None:
            self.gotImage = True
            # create image from RGB888
            image = pygame.image.frombuffer(fb[2].flat[0:], (fb[0], fb[1]), 'RGB')
            fps = self.Clock.get_fps()
            # blit stuff
            self.screen.blit(image, (0, 0))
            # self.screen.blit(self.font.render("FPS %.2f"%(fps), 1, (255, 0, 0)), (0, 0))

            # update display
            pygame.display.flip()
            
            return image
        else:
            return None

    def getOpenMVSerial(self):
        # a = pyopenmv.tx_buf_len()
        out = pyopenmv.tx_buf(pyopenmv.tx_buf_len()).decode()[:-2]
        if "fps" in out:
            try:
                self.omv_fps = out[out.find('fps')+6:out.find('fps')+10]
            except:
                self.omv_fps = 50.0
            # print(self.omv_fps)
        # print(out)
        return out
    def destroy(self):
        # self.waveformroot.destroy()
        self.root.destroy()
        pygame.quit()
        if self.ser.OMVConnected == True:
            pyopenmv.disconnect()  
        sys.exit()

import serial.tools.list_ports

serialports = serial.tools.list_ports.comports()
if __name__ == '__main__':
    Connections = EstablishConnections()

    root = tk.Tk()
    root.configure(background='white')
    root.resizable(0,0)
    gui = HELPER_GUI(root, Connections)
    
    gui.run_update()
    gui.mainloop()

