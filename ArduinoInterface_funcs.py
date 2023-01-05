from asyncio import WriteTransport
from serial import Serial
import tkinter as tk
import threading
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import numpy as np
import json
import serial.tools.list_ports
from math import floor
import sys
class CS3D_GUI:
    def __init__(self, root, ser=None):
        self.root = root
        self.root.configure(background='white')
        self.root.resizable(0,0)
        root.title('Cytostretcher 3D')
        
        self.root.protocol("WM_DELETE_WINDOW", self.destroy)
        
        self.ser = ser
        if self.ser != None:
            self.conn = ser.conn
        self.t = 0
        self.activeMotor = 1
        self.WellLocations = [0, 4000, 7900, 11800]
        # self.WellLocations = [0, 7775, 15550, 23325]
        self.wellLabels = ['D','C','B','A']
        self.motorFrames = []
        self.moveToButtons, self.retractButton,self.startStopIndividualMotor, self.waveformButton = [], [], [], []
        self.positions, self.positionLabels = [], []
        self.freqsLong, self.freqLabel = [], []
        self.dists, self.distLabel = [], []
        self.dist_num = [0,0,0,0]
        self.enable, self.enableLabel, self.motorEnableState = [], [], [0,0,0,0]
        self.passive_len = [0,0,0,0]
        self.desiredStretch = tk.StringVar(self.root, '20')
        self.stretchEntry = []
        self.desiredStretchMotor = []
        self.setTissueStretch = [10,10,10,10]
        self.positionHistory = []
        self.pos = tk.IntVar(self.root, 0)
        self.stretchHistory = []
        self.stretch = 0
        self.foundMax = 0
        self.maxs = ""
        self.well = 0
        self.t_camera = 0
        self.SerialInput = tk.StringVar(self.root, value='1')
        self.output = tk.StringVar(self.root, value='')

        self.saveDataFlag = False
        self.applyFeedback = "Cyclic"
        self.feedbackActiveFlag = [True,True,True,True]
        self.counter = 0
        self.almostCounter = 0
        self.reachedDesiredStretch = [False, False, False, False]

        self.beginMotorInitFlag = [0,0,0,0]
        self.cameraInited = [0,0,0,0]
        self.motorInited = [False, False, False, False]
        self.rise = []
        self.hold = []
        self.fall = []
        self.freqs = []
        self.wfmotor = 0
        
        self.waveformroot = tk.Toplevel(self.root)
        self.waveformroot.configure(background='white')
        self.waveformroot.withdraw()
        my_dpi = 75
        fig = Figure()
        
        self.plot1 = fig.add_subplot(111)
        
        self.canvas = FigureCanvasTkAgg(fig, master = self.waveformroot)  
        self.canvas.draw()
        # placing the toolbar on the Tkinter window
        self.canvas.get_tk_widget().place(x=150,y=0,width=150, height=200)
        fig.tight_layout()
        self.old_vals = []
        
        self.TextBox = tk.LabelFrame(self.root, borderwidth=2, bg='white', relief='groove')
        self.TextBox.place(x=500,y=100, width=300, height=200)
        self.outText = tk.StringVar(self.TextBox, '')
        self.message = tk.Label(self.TextBox, textvariable=self.outText, justify=tk.LEFT, bg='white')
        self.message.place(x=0,y=0,width=180,height=180)
        for i in range(4):
            def moveTo(motor=i):
                string = 'C'+str(motor)+','+str(self.WellLocations[motor])
                self.writeToSerial((string+'\n'))
                # self.conn.write((string+'\n').encode())
                self.moveToButtons[int(self.well)].configure(state='normal')
                self.moveToButtons[int(self.well)].configure(text='Move Camera\nto Row {}'.format(self.wellLabels[int(self.well)]))
                self.motorFrames[int(self.well)].configure(borderwidth=2)
                self.well = str(motor)
                self.moveToButtons[int(self.well)].configure(text='Camera Under\nRow {}'.format(self.wellLabels[int(self.well)]))
                self.moveToButtons[int(self.well)].configure(state='disabled')
                self.motorFrames[int(self.well)].configure(borderwidth=7)
                self.counter = 0
                self.almostCounter = 0
                
                self.InitCameraButton.place(x=325, y=motor*105, width=100, height=40)
                self.homeMotorButton.place(x=325, y=motor*105+40, width=100, height=25)
                self.adjUpButton.place(x=300, y=motor*105+40, width=25, height=25)
                self.adjDownButton.place(x=425, y=motor*105+40, width=25, height=25)
                self.messageScreen('Moving Camera to Row {}'.format(self.wellLabels[int(self.well)]))
            def retractMotor(motor = i):
                string = 'X'+str(motor)
                self.writeToSerial(string+'\n')
                self.messageScreen('Retracting Motor for Row {}.'.format(self.wellLabels[motor]))
            def EnableMotor(motor = i):
                if self.motorEnableState[motor] == 1 and self.motorInited[motor] == True:
                    string = 'M'+str(motor)
                    self.writeToSerial(string+'\n')
                    self.messageScreen('Starting stretch for Row {}.'.format(self.wellLabels[motor]))
                    self.startStopIndividualMotor[motor].configure(text='Stop')
                    self.startStopIndividualMotor[motor].configure(borderwidth=5)
                elif self.motorEnableState[motor] == 0:
                    string = 'M'+str(motor)
                    self.writeToSerial(string+'\n')
                    self.messageScreen('Stopping stretch for Row {}.'.format(self.wellLabels[motor]))
                    self.startStopIndividualMotor[motor].configure(text='Start')
                    self.startStopIndividualMotor[motor].configure(borderwidth=2)
                    
                else:
                    self.messageScreen('Error occured. Cannot start stretch for Row {}.'.format(self.wellLabels[motor]))
            def changeStretch(motor=i):
                x = self.desiredStretchMotor[motor].get()
                self.desiredStretchMotor[motor].set(floor(x))
                self.stretchEntry[motor].update()
                if x > 25:
                    self.messageScreen('Stretching to more than 25% is disabled.')
                    self.desiredStretchMotor[motor].set(floor(self.setTissueStretch[motor]))
                    self.stretchEntry[motor].update()
                elif x > 15:
                    self.messageScreen('Setting stretch to {}% may not be possible. Check if the tissue reaches {}%, and adjust if necessary.'.format(x,x))
                    self.setTissueStretch[motor] = self.desiredStretchMotor[motor].get()
                
                elif x < 0:
                    self.messageScreen('Cannot set to negative stretch.')
                    self.desiredStretchMotor[motor].set(floor(self.setTissueStretch[motor]))
                    self.stretchEntry[motor].update()
                else:
                    self.messageScreen('Updated stretch for Row {}: {}'.format(self.wellLabels[motor], x))    
                    self.setTissueStretch[motor] = self.desiredStretchMotor[motor].get()
                
                print(self.setTissueStretch)
                pass
            def openWaveformDialog(motor=i):
                self.waveformroot.deiconify()
                self.wfmotor = motor
                self.old_vals = [self.rise[motor].get(), self.hold[motor].get(), self.fall[motor].get(), self.freqs[motor].get(), self.desiredStretchMotor[motor].get()]
                self.riseEntry.configure(textvariable=self.rise[motor])
                self.holdEntry.configure(textvariable=self.hold[motor])
                self.fallEntry.configure(textvariable=self.fall[motor])
                self.freqEntry.configure(textvariable=self.freqs[motor])
                self.stretchEntry.configure(textvariable=self.desiredStretchMotor[motor])
                self.waveformtitle.configure(text='Row {}'.format(self.wellLabels[motor]))
                
                # self.waveformroot.mainloop()
                pass
            self.motorFrames.append(tk.Frame(self.root, background='white', borderwidth=2, relief='groove', width=500, height=100))
            
            self.moveToButtons.append(              self.addButtonWidget(self.motorFrames[-1], text='Move Camera\nto Row '+self.wellLabels[i], command=moveTo,location=[0,0,125,40], locationoption='abs'))
            self.retractButton.append(              self.addButtonWidget(self.motorFrames[-1], text='Retract\nMotor', command=retractMotor,location=[50,60,50,25], locationoption='abs', fontsize=8))
            self.startStopIndividualMotor.append(   self.addButtonWidget(self.motorFrames[-1], text='Start', command=EnableMotor, location=[0,60,50,25], locationoption='abs', fontsize=8, state='disabled'))
            self.startStopIndividualMotor[-1].configure(bg='gray')

            self.rise.append(tk.IntVar(self.waveformroot, 40))
            self.hold.append(tk.IntVar(self.waveformroot, 60))
            self.fall.append(tk.IntVar(self.waveformroot, 75))
            self.freqs.append(tk.DoubleVar(self.waveformroot, 1))
            self.desiredStretchMotor.append(tk.IntVar(self.waveformroot, value=10))
            
            self.waveformButton.append(self.addButtonWidget(self.motorFrames[-1], text='Shape', command=openWaveformDialog, location=[105,60,45,25], fontsize=10))
            self.addLabelWidget(self.motorFrames[-1], text=": stretch, Hz, wave shape", fontsize=8, location=[150, 60, -1, 25])
            # tk.Label(self.motorFrames[-1], text=" (stretch %, frequency, waveform, etc.)", font=tk.font.Font(size=8)).place(x=155, y=60)
            self.positions.append(tk.StringVar(self.motorFrames[-1], value='Position: 0 steps'))
            self.positionLabels.append(tk.Label(self.motorFrames[-1], background='white', foreground='black', textvariable=self.positions[-1]))
            
            self.dists.append(tk.StringVar(self.motorFrames[-1], value='Distance: 20 steps'))
            self.distLabel.append(tk.Label(self.motorFrames[-1], textvariable=self.dists[i], foreground='black', background='white'))
            
            self.freqsLong.append(tk.StringVar(self.motorFrames[-1], value='Frequency: 1 Hz'))
            self.freqLabel.append(tk.Label(self.motorFrames[-1], textvariable=self.freqsLong[i], foreground='black', background='white'))
            
            self.enable.append(tk.StringVar(self.motorFrames[-1], value='Disabled'))
            self.enableLabel.append(tk.Label(self.motorFrames[-1], textvariable=self.enable[i], foreground='black', background='white'))
            
            # self.moveToButtons[-1].configure(command=moveTo)
            # self.retractButton[-1].configure(command=retractMotor)

            self.motorFrames[-1].place(bordermode=tk.OUTSIDE, height=100, width=300, x=0, y=i*105)
            # # self.moveToButtons[-1].place(relwidth=0.3, relheight=0.5, relx=0.7, rely=0.05)
            # # self.retractButton[-1].place(relwidth=0.3, relheight=0.2, relx=0.7, rely=0.85)
            # self.positionLabels[-1].place(relx=0.1, rely=0)
            # self.freqLabel[-1].place(relx=0.1, rely=0.2)
            # self.distLabel[-1].place(relx=0.1, rely=0.4)
            # self.enableLabel[-1].place(relx=0.0, rely=0.6)

        self.resetCameraButton  = self.addButtonWidget(self.root, command=self.resetCamera, text='Reset Camera\nPosition', location=[100,430,100,50], locationoption='abs', fontsize=10)
        self.retractAllButton   = self.addButtonWidget(self.root, command=self.retractAllMotors, text='Retract All\nMotors', location=[200, 430, 100, 50], locationoption='abs', fontsize=10)
        self.stopAllButton      = self.addButtonWidget(self.root, command=self.stopAllMotors, text='Stop All\nMotors', location=[0,430,100,50], locationoption='abs',fontsize=10)
        self.homeMotorButton    = self.addButtonWidget(self.root, command=self.initializeMotor, text='Home Motor', location=[400, 300, 100, 25], locationoption='abs', fontsize=10)
        self.adjUpButton        = self.addButtonWidget(self.root, command=lambda: self.sendAdj(0), text='-', location=[380, 302.5, 20, 20])
        self.adjDownButton      = self.addButtonWidget(self.root, command=lambda: self.sendAdj(1), text='+', location=[500, 302.5, 20, 20])
        # self.StartStopMotor     = self.addButtonWidget(self.root, command=self.EnableDisable, text='Start Stretching', location=[400,325,100,25], fontsize=10, state='disabled')
        self.InitCameraButton   = self.addButtonWidget(self.root, command=self.INITCAMERA, text='Initialize\nFeedback', location=[400,260,100,40], fontsize=10)
        # self.feedbackButton     = self.addButtonWidget(self.root, command=self.toggleFeedback, text='Enable\nFeedback', location=[400, 400, 100,40], fontsize=10, state='disabled')

        self.closeButton        = self.addButtonWidget(self.root, command=self.destroy, text='Close\nProgram', location=[700, 400, 100, 100])
        self.Output = tk.Entry(self.root, textvariable=self.output)
        self.Output.bind('<Return>', func=lambda val: self.sendOutput())
        self.Output.place(x=500, y=300)

        self.waveformroot.geometry("300x250")
        
        self.riseEntry      = self.addEntryWidget(self.waveformroot, textvariable=self.rise[0], location=[10,40,30,25], fontsize=10)
        self.holdEntry      = self.addEntryWidget(self.waveformroot, textvariable=self.hold[0], location=[10,70,30,25], fontsize=10)
        self.fallEntry      = self.addEntryWidget(self.waveformroot, textvariable=self.fall[0], location=[10,100,30,25], fontsize=10)
        self.freqEntry      = self.addEntryWidget(self.waveformroot, textvariable=self.freqs[0], location=[10,130, 30, 25], fontsize=10)
        self.stretchEntry   = self.addEntryWidget(self.waveformroot, textvariable=self.desiredStretchMotor[0], location=[10,160,30,25], fontsize=10)
        
        tk.Scale(self.root, from_ = -1000, to=1000, variable = self.pos, command=self.sendPosition).pack()
        
        self.plotWaveform()
        self.waveformtitle = self.addLabelWidget(self.waveformroot, text='Row {}'.format(self.wellLabels[0]), location=[0,0,150,35], fontsize=12, justify=tk.CENTER)
        # self.waveformtitle = tk.Label(self.waveformroot, text='Row {}'.format(self.wellLabels[0]), justify=tk.CENTER, font=tk.font.Font(size=14))
        # self.waveformtitle.place(x=0,y=0,width=150)
        def sendWaveform(motor):
            if motor != -1:
                motorsToCheck = [motor]
            else:
                motorsToCheck = range(4)
            
            for motor in motorsToCheck:
                x = self.desiredStretchMotor[motor].get()
                self.desiredStretchMotor[motor].set(floor(x))
                self.stretchEntry.update()
                continueFlag = False
                if x > 25:
                    self.messageScreen('Row {}: Stretching to more than 25% is disabled.'.format(self.wellLabels[motor]))
                    self.desiredStretchMotor[motor].set(floor(self.setTissueStretch[motor]))
                    self.stretchEntry[motor].update()
                elif x > 15:
                    self.messageScreen('Row {}: Setting stretch to {}% may not be possible. Check if the tissue reaches {}%, and adjust if necessary.'.format(self.wellLabels[motor],x,x))
                    self.setTissueStretch[motor] = self.desiredStretchMotor[motor].get()
                    continueFlag = True
                elif x < 0:
                    self.messageScreen('Row {}: Cannot set to negative stretch.'.format(self.wellLabels[motor]))
                    self.desiredStretchMotor[motor].set(floor(self.setTissueStretch[motor]))
                    self.stretchEntry[motor].update()
                else:
                    self.messageScreen('Row {}: Stretch set to {}%.'.format(self.wellLabels[motor], x))    
                    self.setTissueStretch[motor] = self.desiredStretchMotor[motor].get()
                    continueFlag = True
                
                if continueFlag == True:
                    self.waveformroot.withdraw()
                    string = "S{},{},{},{}".format(motor, self.rise[motor].get(), self.hold[motor].get(), self.fall[motor].get())
                    self.writeToSerial(string+'\n')
                    self.messageScreen("Row {}: Setting shape: [{}, {}, {}].".format(self.wellLabels[motor], self.rise[motor].get(), self.hold[motor].get(), self.fall[motor].get()))
                    string = "F{},{}".format(motor, self.freqs[motor].get())
                    self.writeToSerial(string+'\n')
                    self.messageScreen("Row {}: Setting frequency: {}.".format(self.wellLabels[motor], self.freqs[motor].get()))
        self.addLabelWidget(self.waveformroot, text=': Rise time (t/T)', location=[40,40,-1,-1], fontsize=10, justify=tk.LEFT)
        self.addLabelWidget(self.waveformroot, text=': Hold time (t/T)', location=[40,70,-1,-1], fontsize=10, justify=tk.LEFT)
        self.addLabelWidget(self.waveformroot, text=': Fall time (t/T)', location=[40,100,-1,-1], fontsize=10, justify=tk.LEFT)
        self.addLabelWidget(self.waveformroot, text=': Frequency (Hz)', location=[40,130,-1,-1], fontsize=10, justify=tk.LEFT)
        self.addLabelWidget(self.waveformroot, text=': Stretch (%)', location=[40,160,-1,-1], fontsize=10, justify=tk.LEFT)

        def disable_event():
            self.waveformroot.withdraw()
            self.rise[self.wfmotor].set(self.old_vals[0])
            self.hold[self.wfmotor].set(self.old_vals[1])
            self.fall[self.wfmotor].set(self.old_vals[2])
            self.freqs[self.wfmotor].set(self.old_vals[3])
            self.desiredStretchMotor[self.wfmotor].set(self.old_vals[4])
            pass
        self.waveformroot.protocol("WM_DELETE_WINDOW", disable_event)
        
        self.closeButton = self.addButtonWidget(self.waveformroot, text='Program Motor', location=[50,205,100,25], fontsize=12, command=lambda: sendWaveform(self.wfmotor))
        self.closeButtonAll = self.addButtonWidget(self.waveformroot, text='Program All Motors', location=[155, 205, 150, 25], fontsize=12, command=lambda: sendWaveform(-1))
        
        # self.EnableDisableButton = tk.Button(self.root, borderwidth=2, relief='groove', text="Enable/Disable Motor", command=self.EnableDisable)
        self.ResetButton = tk.Button(self.root, borderwidth=2, relief='groove', text='Zero Motor Position', command=self.ResetMotor)
        # self.feedbackButton = tk.Button(self.root, text='Enable Feedback', borderwidth=2, relief='groove', command=self.toggleFeedback)
        
        # self.EnableDisableButton.place(relx=0.5,rely=0.6)
        # self.ResetButton.pack()
        # self.feedbackButton.place(relx=0.6, rely=0.55)
        self.desiredStretchEntry = tk.Entry(self.root, relief='groove',borderwidth=2, textvariable=self.desiredStretch)
        # self.desiredStretchEntry.place(relx=0.55, rely=0.55, width=20)
        # tk.Label(self.root, text="Stretch:").place(relx=0.49, rely=0.55)
        # tk.Label(self.root, text="%").place(relx=0.575, rely=0.55, width=10)
        self.textLabel = tk.Label(self.root, textvariable=self.SerialInput, fg='black')
        # self.textLabel.pack()
        
        self.saveButton = tk.Button(self.root, relief='groove', borderwidth=2, text="Save Data", command=self.saveData)
        # self.saveButton.pack()
        self.saveLabel = tk.Label(self.root, text='')
        # self.saveLabel.pack()
        self.LED = tk.IntVar(self.root, 100)
        
        
        # tk.Scale(self.root, command=self.LEDBrightness, bg='white', relief='groove', from_=0, to=100, variable=self.LED, orient='horizontal').place(x=300, y=400)
        
        self.motorFrames[self.well].configure(borderwidth=7)

        magic = '0'
        self.t0 = 0
        string, magic = self.readFromSerial()
        print(magic)
        if magic == '-33':
            self.t0 = int(string.split(';')[1])
            print(string)
            self.well = string.split(';')[-1].split('&')[-1]
        else:
            self.well = '0'
            # self.LED.set(string.split(';')[-1].split('&')[-1][:-1])
        print(self.well)
        self.moveToButtons[int(self.well)].invoke()
            
            # self.moveToButtons[int(self.well)].configure(state='disabled')
            # self.motorFrames[int(self.well)].configure(borderwidth=7)
                
        # while magic != '-33' and magic != -1:
        #     string, magic = self.readFromSerial()
        #     self.root.after(1)
        #     self.t0 = int(string.split(';')[1])

        # self.fig = Figure(figsize=(2,2))
        # self.ax = self.fig.add_subplot(111)
        # self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        # self.canvas.draw()
        # self.canvas.get_tk_widget().place(relx=0.45, rely=0.6, relwidth=0.3, relheight=0.4)
    def LEDBrightness(self, value):
        string = "LED,{}".format(self.LED.get())
        self.writeToSerial(string)
        # print(string)
    def messageScreen(self, string,**kwargs):
        # self.outText.set(self.outText.get()+'\n'+str(self.t-self.t0)+': '+string)
        print(string, **kwargs)
    
    def addButtonWidget(self, root, command, text, location, locationoption='abs', fontsize=12, **kwargs):
        font = tk.font.Font(size=fontsize)
        button = tk.Button(root, command=command, text=text, background='white', borderwidth=2, relief='groove', font=font, **kwargs)
        if locationoption=='rel':
            button.place(relx=location[0], rely=location[1], relwidth=location[2], relheight=location[3])
        elif locationoption=='abs':
            button.place(x=location[0], y=location[1], width=location[2], height=location[3])
        elif locationoption=='pack':
            button.pack()
        
        return button
    def addLabelWidget(self, root, text, location, locationoption='abs', fontsize=12, **kwargs):
        font=tk.font.Font(size=fontsize)
        label = tk.Label(root, text=text, background='white', font=font, **kwargs)
        if locationoption=='rel':
            if location[2] == -1 and location[3] == -1:
                label.place(relx=location[0], rely=location[1])
            elif location[2] == -1:
                label.place(relx=location[0], rely=location[1], relheight=location[3])
            elif location[3] == -1:
                label.place(relx=location[0], rely=location[1], relwidth=location[2])
            else:
                label.place(relx=location[0], rely=location[1], relwidth=location[2], relheight=location[3])
        elif locationoption=='abs':
            if location[2] == -1 and location[3] == -1:
                label.place(x=location[0], y=location[1])
            elif location[2] == -1:
                label.place(x=location[0], y=location[1], height=location[3])
            elif location[3] == -1:
                label.place(x=location[0], y=location[1], width=location[2])
            else:
                label.place(x=location[0], y=location[1], width=location[2], height=location[3])
        elif locationoption=='pack':
            label.pack()
        elif locationoption=='grid':
            label.grid(location[0], location[1])
        
        return label
    def addEntryWidget(self, root, textvariable, location, locationoption='abs', fontsize=12, **kwargs):
        font = tk.font.Font(size=fontsize)
        entry = tk.Entry(root, relief='groove',borderwidth=2, background='white', textvariable=textvariable, font=font)
        if locationoption=='rel':
            entry.place(relx=location[0], rely=location[1], relwidth=location[2], relheight=location[3])
        elif locationoption=='abs':
            entry.place(x=location[0], y=location[1], width=location[2], height=location[3])
        elif locationoption=='pack':
            entry.pack()
        
        return entry
    def resetCamera(self):
        self.writeToSerial('V'+'\n')  
        self.moveToButtons[int(self.well)].configure(state='normal')
        self.moveToButtons[int(self.well)].configure(text='Move Camera\nto Row {}'.format(self.wellLabels[int(self.well)]))
        self.motorFrames[int(self.well)].configure(borderwidth=2)
        self.well = str(0)
        self.moveToButtons[int(self.well)].configure(text='Camera Under\nRow {}'.format(self.wellLabels[int(self.well)]))
        self.moveToButtons[int(self.well)].configure(state='disabled')
        self.motorFrames[int(self.well)].configure(borderwidth=7)
        self.counter = 0
        self.almostCounter = 0
        self.messageScreen('Resetting Camera Position. Now under Row {}'.format(self.wellLabels[int(self.well)]))   
    def initializeMotor(self):
        string = "MOTORINIT&" + str(self.well)
        self.messageScreen('Initializing Motor...', end='')
        self.writeToSerial("INIT"+'\n')
        self.root.after(1000)
        self.writeToSerial(string+'\n')
        self.messageScreen('Initializing Successful.')
    def retractAllMotors(self):
        for i in range(4):
            self.writeToSerial('X'+str(i)+'\n')
            self.root.after(1)     
    def readFromSerial(self):
        string = ''
        magic = -1
        
        if self.conn != None:
            if self.conn != []:
                string = self.conn.readline().decode('utf-8')[:-2]
                magic = string.split(';')[0]
                self.conn.flushInput()
        
        return string, magic
    def writeToSerial(self, string):
        if self.conn != None:
            self.conn.write(string.encode())
        # print((str(self.t)+': '+string))
    def stopAllMotors(self):
        for i in range(4):
            self.writeToSerial('O'+str(i)+',0\n')
            self.root.after(1)
        self.messageScreen('Stopping All Motors.')
        
    def toggleFeedback(self):
        if self.feedbackActiveFlag[int(self.well)] == True:
            self.feedbackActiveFlag[int(self.well)] = False
            self.feedbackButton.configure(borderwidth=2)
        else:
            self.feedbackActiveFlag[int(self.well)] = True
            self.feedbackButton.configure(borderwidth=5)
    def saveData(self):
        if self.saveDataFlag == False:
            with open('samplefile.txt', 'w') as f:
                f.write('{},{},{}\n'.format('T','motor','stretch'))
            self.saveDataFlag = True
            self.saveLabel.configure(text='Saving')
            self.saveLabel.update()
        else:
            self.saveDataFlag = False
            self.saveLabel.configure(text='')
            self.saveLabel.update()
    def INITCAMERA(self):
        string = 'INIT'
        self.writeToSerial(string+'\n\n')   
    def plotWaveform(self):
        # the figure that will contain the plot
        
        try:
            rise = self.rise[self.wfmotor].get()
            hold = self.hold[self.wfmotor].get()
            fall = self.fall[self.wfmotor].get()
            stretch = self.desiredStretchMotor[self.wfmotor].get()
            freq = self.freqs[self.wfmotor].get()
        except:
            return
        
        if hold < rise+10 or fall < hold+10:
            return
        if fall > 90:
            return
        if rise < 10:
            return
        # list of squares
        if freq != 0:
            t = 1/freq
        elif freq > 4:
            freq = 4
            t = 1/freq
        elif freq <= 0:
            t=0.25
        x = [0, rise, hold, fall, 100]
        y = [0, stretch, stretch, 0, 0]
        # adding the subplot
        
        # plotting the graph
        self.plot1.clear()
        self.plot1.plot(x,y)
        self.plot1.set_xticks(x)
        self.plot1.grid(True)
        labels = ['', 'RISE','HOLD','FALL','T={}s'.format(round(t,2))]
        self.plot1.set_xticklabels(["{}\n{}".format(int(x[i]), labels[i]) for i in range(len(labels))], fontsize=6)
        self.plot1.set_ylim([0, 26])
        self.plot1.set_yticks([0,stretch])
        self.plot1.set_yticklabels(["{}%".format([0,stretch][i]) for i in range(len([0,stretch]))], fontsize=6)
        self.canvas.draw()
        # creating the Tkinter canvas
        # containing the Matplotlib figure      
    def ResetMotor(self):
        string = 'R'+str(self.well)
        self.writeToSerial(string+'\n')
        self.pos.set(0)
        self.Slider.update()
    def sendAdj(self, adj):
        string = 'A'+str(self.well)+','+str(adj)
        self.writeToSerial(string+'\n')
    def sendPosition(self, value=None):
        string = 'O'+str(self.well)+','+str(self.pos.get())
        self.writeToSerial(string+'\n')
    def sendOutput(self):
        self.writeToSerial(self.output.get()+'\n')
        self.output.set('')
        self.Output.update()
    def EnableDisable(self):
        if self.motorEnableState[self.well] == 1 and self.motorInited[self.well] == True:
            string = 'M'+str(self.well)
            self.writeToSerial(string+'\n')
        elif self.motorEnableState[self.well] == 0:
            string = 'M'+str(self.well)
            self.writeToSerial(string+'\n')
        pass
    def update(self):
        if self.waveformroot.winfo_viewable():
            self.plotWaveform()
        if self.ser == None:
            string = '1'
        else:
            string, magic = self.readFromSerial()
            # print(self.feedbackActiveFlag)
            self.values = string.split(';')
            self.motorValues = []
            magic = self.values.pop(0)
            # print(self.t0)
            # print(self.values)
            if magic == 'HELPER':
                pass
            elif magic == '-33':
                self.t = int(self.values.pop(0))
                self.t_camera, self.well, self.MotorInitWell, self.recievedMotorInitHandshake, self.stretch, self.foundMax, self.max_stretch = tuple(self.values.pop(0).split('&'))
                self.cameraVals = self.values[-1].split('&')
                if self.cameraVals[0] == '-1':
                    """ Camera Moving """
                    self.InitCameraButton.configure(state='disabled')
                    self.homeMotorButton.configure(state='disabled')
                else:
                    self.InitCameraButton.configure(state='normal')
                    self.homeMotorButton.configure(state='normal')

                self.well = self.cameraVals[-1][:-1]
                
                for i in range(4):
                    motorID, motorT, position, dist, freq, passive_len, MotorInited, CameraInited, enableMotorState, beginMotorInitFlag = tuple(self.values[0].split('&'))
                    
                    # print("{},{}".format(self.beginMotorInitFlag[i],beginMotorInitFlag))
                    if self.beginMotorInitFlag[i] == 1:
                        if int(beginMotorInitFlag) == 0:
                            self.messageScreen('Motor Initialization Success.')
                    
                    # if self.cameraInited[i] == 0:
                    #     if int(CameraInited) == 1:
                    #         self.messageScreen('Camera Initilization Success.')
                    self.beginMotorInitFlag[i] = int(beginMotorInitFlag)
                    self.motorInited[i] = int(MotorInited)
                    # self.motorInited[i] = 1

                    
                    self.cameraInited[i] = int(CameraInited)
                    # self.cameraInited[i] = True
                    if self.cameraInited[i]==1 and i == int(self.well):
                        # print(1)
                        self.InitCameraButton.configure(text='Re-initialize\nFeedback')
                        # self.feedbackButton.configure(state='normal')
                    elif i == int(self.well):
                        # print(2)
                        self.InitCameraButton.configure(text='Initialize\nFeedback')
                        # self.feedbackButton.configure(state='disabled')
                    
                    # if self.feedbackActiveFlag[i] == True and i == int(self.well):
                    #     # self.feedbackButton.configure(text='Disable\nFeedback')
                    # elif self.feedbackActiveFlag[i] == False and i == int(self.well):
                        # self.feedbackButton.configure(text='Enable\nFeedback')
                    
                    if self.motorInited[i] == 1 and i == int(self.well):
                        self.homeMotorButton.configure(text='Re-home Motor')
                    elif i == int(self.well):
                        self.homeMotorButton.configure(text='Home Motor')
                    
                    if self.motorInited[i] == 1:
                        self.startStopIndividualMotor[i].configure(state='normal')
                        self.startStopIndividualMotor[i].configure(bg='green')
                    else:
                        self.startStopIndividualMotor[i].configure(state='disabled')
                        self.startStopIndividualMotor[i].configure(bg='gray')

                    if self.motorEnableState[i] == 1: # enabled
                        self.startStopIndividualMotor[i].configure(text='Start')
                        self.startStopIndividualMotor[i].configure(borderwidth=2)
                    elif self.motorEnableState[i] == 0:
                        self.startStopIndividualMotor[i].configure(text='Stop')
                        self.startStopIndividualMotor[i].configure(borderwidth=5)
                    
                    # if self.beginMotorInitFlag == 1:
                    #     self.homeMotorButton.configure(state='disabled')
                    # else:
                    #     self.homeMotorButton.configure(state='normal')
                    self.values.pop(0)
                    if self.feedbackActiveFlag[int(self.well)] == True and int(self.well) == i:
                        self.positions[i].set('Stretch: '+str(round(float(self.stretch),1))+' %')
                    else:
                        self.positions[i].set('Current position: '+position+' steps')
                    self.freqsLong[i].set('Frequency: '+freq+' Hz')
                    self.dists[i].set('Distance: '+dist+' steps')
                    self.dist_num[i] = int(dist)
                    self.passive_len[i] = int(passive_len)
                    self.motorEnableState[i] = int(enableMotorState)
                    # if int(motorEnable) == 1: # disabled
                    #     self.enable[i].set('Disabled')
                    #     self.enableLabel[i].configure(foreground='black')
                    # else: # enabled
                    #     self.enable[i].set('Enabled')
                    #     self.enableLabel[i].configure(foreground='red')
                    
                    val = 'Motor Homed: '
                    if int(MotorInited) == 1: val += 'Y, '
                    else: val += 'N, ' 
                    val += 'Camera Inited: '
                    if int(CameraInited) == 1: val += 'Y'
                    else: val += 'N'
                    if self.feedbackActiveFlag[i] == False:
                        val+="\nFeedback: Not Active"
                    elif self.feedbackActiveFlag[i] == True:
                        if self.motorEnableState[i] == 1:
                            val += "\nMotor Off"
                        else:
                            if self.reachedDesiredStretch[i] == False:
                                val+="\nFeedback: N"
                            else:
                                val+="\nFeedback: Y"
                    # if int(MotorInited) == 1:
                    #     self.EnableDisableButton.configure(state='normal')
                    # else:
                    #     self.EnableDisableButton.configure(state='disabled')
                    self.enable[i].set(val)
                    
                    # print("{0},{1}".format(self.motorT[i-n_before_motors], self.positions[i-n_before_motors].get()), end=' // ')
                    
                    self.positionLabels[i].update()
                    # print(' // ', end=' ')
                # print(self.motorValues)
                # print(' ')
                
                # print(self.stretchHistory)
                # print(self.stretch)
                # self.stretchHistory.append(float(self.stretch))
                self.positionHistory.append(float(self.positions[int(self.well)].get().split(': ')[1].split(' ')[0]))

                if len(self.positionHistory) > 100:
                #     self.stretchHistory.pop(0)
                    self.positionHistory.pop(0)

                # self.ax.clear()
                # self.ax.plot(self.positionHistory, self.stretchHistory)
                # self.canvas.draw()

                if self.feedbackActiveFlag[int(self.well)] == True:
                    self.applyFeedbackFunc()
                else:
                    pass
                    # val = "D"+str(self.well)+','+str(self.dist_num[self.well])
                    # print(val)

                
            # print([self.positions[i].get() for i in range(4)])
            if self.saveDataFlag == True:
                with open('samplefile.txt', 'a') as f:
                    f.write("{},{},{}\n".format(self.t, self.positions[int(self.well)].get()[10:-5], self.stretch))
            
            # print("{}: {}".format(len(self.stretchHistory), self.stretchHistory))
            # print("{}: {}".format(len(self.positionHistory), self.positionHistory))
            # print(string)
            # print("{}, {}".format(self.positions[self.well-1].get(),self.stretch))
            # print(self.positions[0].get().split(': ')[1].split(' ')[0])
            # print(self.well)
            # print(string)
        if self.conn != None:
            self.conn.flush()
        self.root.after(1, self.update)    
    def applyFeedbackFunc(self):
        if self.foundMax=='1' and self.cameraVals[0] != '-1' and self.motorEnableState[int(self.well)] == 0:
            # print(str(self.t)+':', end='')
            self.max_stretch_aslist = json.loads(self.max_stretch)[-1]
            
            max_stretch_active = round(100*np.mean(self.max_stretch_aslist)/self.passive_len[int(self.well)],1)
            # print(np.mean(self.max_stretch_aslist))
            
            diff = max_stretch_active - int(self.setTissueStretch[int(self.well)])
            self.reachedDesiredStretch[int(self.well)] = False
            scalar = 5
            if diff > 2: 
                self.counter = 0
                self.almostCounter = 0
                new_dist = round(1 + abs(diff*0.5))
            elif diff < -2:
                self.counter = 0
                self.almostCounter = 0
                new_dist = round(1 + abs(diff*0.5))
            elif diff > 0.5:
                self.counter = 0
                self.almostCounter += 1
                new_dist = 1
            elif diff < -0.5:
                self.counter = 0
                self.almostCounter += 1
                new_dist = 1
            
            else:
                new_dist = 0
                self.counter += 1
                self.almostCounter += 1
                if self.counter > 2 or self.almostCounter > 4:
                    self.reachedDesiredStretch[int(self.well)] = True
            
            new_dist = -1*np.sign(diff)*new_dist*scalar
            val = "D"+str(self.well)+','+str(int(self.dist_num[int(self.well)]+new_dist))
            print(val)
            self.writeToSerial(val+'\n')
        pass
    def mainloop(self): # blocking
        self.root.mainloop()
    def run_update(self):
        self.thr = threading.Thread(target=self.update())
        self.thr.start() # run update function in background
    def destroy(self):
        self.waveformroot.destroy()
        self.root.destroy()
        sys.exit()

class EstablishConnection:
    def __init__(self, portlist):
        self.portlist = portlist
        self.portNames, self.descs, self.hwids = [],[],[]

        for port, desc, hwid in sorted(portlist):
            self.portNames.append(port)
            self.descs.append(desc)
            self.hwids.append(hwid)
        
        self.conn = None
        self.ActiveSerial = True
        k=0
        for port in self.portNames:
            if 'Bluetooth' in port:
                k = k+1
                continue
            elif 'OpenMV' in port or 'OpenMV' in self.descs[k]:
                k = k+1
                continue
            else:  
                try:
                    self.conn = Serial(port, baudrate=9600, timeout=1)
                    self.ActiveSerial = True
                    print('Connected to Arduino! ')
                    break
                except:
                    print('Tried to connect but failed.  Trying another or exiting.')
                    k = k+1
                    continue
        
        if type(self.conn) == list:
            print('No Serial port found.')
            self.ActiveSerial = False
    
    def close(self):
        self.conn.close()

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.figure as figure
from matplotlib.backend_bases import MouseEvent


# class DraggablePlot():
#     """ An example of plot with draggable markers """
#     def __init__(self, parent):

        
#         self._figure, self._axes, self._line = None, None, None
#         self._dragging_point = None
#         self._points = {0:0, 40:50, 60:50, 70:0}
        
#         self._init_plot()
#         self._update_plot()
        
        

#     def _init_plot(self):
#         self._figure = figure.Figure()
#         axes = plt.subplot(1, 1, 1)
#         axes.set_xlim(0, 100)
#         axes.set_ylim(0, 100)
#         axes.grid(which="both")
#         self._axes = axes

#         self._figure.canvas.mpl_connect('button_press_event', self._on_click)
#         self._figure.canvas.mpl_connect('button_release_event', self._on_release)
#         self._figure.canvas.mpl_connect('motion_notify_event', self._on_motion)
#         plt.show()

#     def _update_plot(self):
#         if not self._points:
#             self._line.set_data([], [])
#             print('1')
#         else:
#             x, y = zip(*sorted(self._points.items()))
#             # print(x)
#             print(y)
#             x_points_list = list(x)
#             if x_points_list[-1] > 90:
#                 x_points_list[-1] = 90
#             x = tuple(x_points_list)
#             for i in range(1,len(x)):
#                 x_distance = x[i]-x[i-1]
#                 if x_distance < 10:
#                     x_list = list(x)
#                     x_list[0] = 0
#                     x_list[i] = x_list[i-1]+10
#                     x = tuple(x_list)
#             # Add new plot
#             if not self._line:
#                 self._line, = self._axes.plot(x, y, "b", marker="o", markersize=10)
#             # Update current plot
#             else:
#                 self._points = dict(zip(x,y))
#                 self._line.set_data(x, y)
        
        
#         self._figure.canvas.draw()
#         print(self._line.get_data())
#         # print(x)
#         self._axes.set_title(x)

#     def _add_point(self, x, y=None):
#         if isinstance(x, MouseEvent):
#             x, y = int(x.xdata), int(x.ydata)
#         self._points[x] = y
#         return x, y

#     def _remove_point(self, x, _):
#         if x in self._points:
#             self._points.pop(x)

#     def _find_neighbor_point(self, event):
#         u""" Find point around mouse position
#         :rtype: ((int, int)|None)
#         :return: (x, y) if there are any point around mouse else None
#         """
#         distance_threshold = 10
#         nearest_point = None
#         min_distance = math.sqrt(2 * (100 ** 2))
#         for x, y in self._points.items():
#             distance = math.hypot(event.xdata - x, event.ydata - y)
#             if distance < min_distance:
#                 min_distance = distance
#                 nearest_point = (x, y)
#         if min_distance < distance_threshold:
#             return nearest_point
#         return None

#     def _on_click(self, event):
#         u""" callback method for mouse click event
#         :type event: MouseEvent
#         """
#         # left click
#         if event.button == 1 and event.inaxes in [self._axes]:
#             point = self._find_neighbor_point(event)
#             if point:
#                 self._dragging_point = point
#                 # print(self._dragging_point)
#                 hold_y = self._dragging_point[1]
#                 # print(hold_y)
#             else:
#                 # self._add_point(event)
#                 pass
#             self._update_plot()
#         # right click
#         # elif event.button == 3 and event.inaxes in [self._axes]:
#         #     point = self._find_neighbor_point(event)
#         #     if point:
#         #         self._remove_point(*point)
#         #         self._update_plot()

#     def _on_release(self, event):
#         u""" callback method for mouse release event
#         :type event: MouseEvent
#         """
#         if event.button == 1 and event.inaxes in [self._axes] and self._dragging_point:
#             self._dragging_point = None
#             self._update_plot()

#     def _on_motion(self, event):
#         u""" callback method for mouse motion event
#         :type event: MouseEvent
#         """
#         if not self._dragging_point:
#             return
#         if event.xdata is None or event.ydata is None:
#             return
#         event.ydata = self._dragging_point[1]
#         if event.xdata > 90:
#             event.xdata = 90
        
#         print(event)
#         self._remove_point(*self._dragging_point)
#         self._dragging_point = self._add_point(event)
        
#         self._update_plot()

location = [0,0,0.25, 0.3]
if __name__ == '__main__':
    serialports = serial.tools.list_ports.comports()
    ser = EstablishConnection(serialports)
    # if ser.ActiveSerial == True:
    #     root = tk.Tk()
    #     root.geometry("800x500")
    #     gui = CS3D_GUI(root, ser)
    #     # HELPER_GUI(gui, ser.conn)

    root = tk.Tk()
    root.geometry("800x500")

    gui = CS3D_GUI(root, ser)
    # root = tk.Tk()
    # gui = CS3D_GUI(root)
    # gui.update()
    gui.run_update()
    gui.mainloop()
