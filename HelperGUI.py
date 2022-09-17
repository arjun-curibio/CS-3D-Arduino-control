from pydoc import Helper
from serial import Serial
import tkinter as tk
import threading
from RangeSlider.RangeSlider import RangeSliderH
from ArduinoInterface_funcs import EstablishConnection

class HELPER_GUI:
    def __init__(self, root, conn=None):
        self.root = root
        root.title('HELPER')

        self.conn = conn

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
        self.rs_post_area = RangeSliderH(self.root, [self.post_area_low, self.post_area_high], min_val = 1000, max_val = 25000,  padX=40, digit_precision='4.0f')
        
        self.s_magnet_extent        = tk.Scale(self.root, from_=0, to=1,    variable=self.magnet_extent,        resolution=0.1, orient='horizontal')
        
        self.rs_magnet_aspectratio   = RangeSliderH(self.root, [self.magnet_aspectratio_low, self.magnet_aspectratio_high],  min_val = 0, max_val = 10, padX=20, digit_precision='0.05f')

        self.s_post_circularity     = tk.Scale(self.root, from_=0.5, to=1,  variable=self.post_circularity,     resolution=0.05, orient='horizontal')
        
        self.rs_magnet_thresh.pack()
        self.rs_post_thresh.pack()
        self.rs_magnet_area.pack()
        self.rs_post_area.pack()
        self.s_magnet_extent.pack()
        self.rs_magnet_aspectratio.pack()
        self.s_post_circularity.pack()
        
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
        print(self.conn.readline().decode('utf-8')[:-2])
        self.conn.flush()

        self.root.after(500, self.update)
    
    def run_update(self):
        self.thr = threading.Thread(target=self.update())
        self.thr.start()

    def mainloop(self):
        self.root.mainloop()
    

import serial.tools.list_ports

serialports = serial.tools.list_ports.comports()
ser = EstablishConnection(serialports)
if ser.ActiveSerial == True:
    root = tk.Tk()
    gui = HELPER_GUI(root, ser.conn)
    ser.conn.write('HELPERTOGGLE\n'.encode())


gui.run_update()
gui.mainloop()

