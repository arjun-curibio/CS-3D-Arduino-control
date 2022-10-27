from asyncio import WriteTransport
from serial import Serial
import tkinter as tk
import threading
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import numpy as np
import json



class CS3D_GUI:
    def __init__(self, root, ser = None):
        self.root = root
        self.root.configure(background='white')
        self.root.tk_setPalette(background='white', foreground='black')
        root.title('Cytostretcher 3D')

        self.ser = ser
        if self.ser != None:
            self.conn = ser.conn
        else:
            self.conn = None
        
        # TIMING VARIABLES
        self.t = 0
        self.t_camera = 0
        self.t_motor = [0,0,0,0]

        # Compare these two to make sure camera and motor are the same
        self.activeMotor = 0
        self.activeFeedback = False
        # MOTOR VARIABLES
        self.freqs = []

        self.WellLocations = [0, 7775, 15550, 23325]
        self.wellLabels = ['D','C','B','A']

        self.activeMotorFrame = tk.Frame(self.root, borderwidth=2, relief='groove', width=500, height=100)
        self.activeMotorLabel = tk.Label(self.activeMotorFrame, text='Active Motor: Row D')
        self.activeMotorLabel.pack()
        
        
        def retractMotor(): # retract active motor
            string = "X"+str(self.activeMotor)
            self.writeToSerial(string)
        def initializeCamera(): # initalize camera under active motor
            string = 'INIT'
            self.writeToSerial(string)
        def initializeMotor(): # home active motor
            string = "MOTORINIT&"+str(self.activeMotor)
            initializeCamera()
            self.root.after(1000)
            self.writeToSerial(string)
            pass
        def enableMotor(): # start stop stretching
            string='M'+str(self.activeMotor)
            self.writeToSerial(string)
            pass
        def toggleFeedback():
            if self.activeFeedback == True:
                self.activeFeedback = False
                self.feedbackButton.configure(borderwidth=2)
                print("DISABLING FEEDBACK (OPEN LOOP)")
            else:
                if len(self.activeFeedbackLabel.get()) > 0:
                    try:    self.desiredStretch = float(self.activeFeedbackLabel.get())
                    except: print("INPUT NUMBERS ONLY."); return

                    if   self.desiredStretch < 5:   print("STRETCH TOO SMALL (minimum = 5%)")
                    elif self.desiredStretch > 30:  print("STRETCH TOO LARGE (maximum = 30%)")
                    elif self.desiredStretch > 18: # Valid, but warning 
                        print("ENABLING FEEDBACK: "+str(self.desiredStretch)+'% ***MAY NOT REACH')
                        self.activeFeedback = True
                        self.feedbackButton.configure(borderwidth=5)
                    elif self.desiredStretch <= 18 and self.desiredStretch >= 5: # Valid
                        print("ENABLING FEEDBACK: "+str(self.desiredStretch)+'%')
                        self.activeFeedback = True
                        self.feedbackButton.configure(borderwidth=5)
                
                else: print("ENTER VALUE TO THE LEFT (numbers only)")
        def adjustUp():
            string = 'A'+str(self.activeMotor)+',0'
            self.writeToSerial(string)
            pass
        def adjustDown():
            string = 'A'+str(self.activeMotor)+',1'
            self.writeToSerial(string)
            pass
        
        
        
        self.initMotorButton = tk.Button(self.activeMotorFrame, relief='groove',borderwidth=2, text='Home Motor', command=initializeMotor)
        self.retractMotorButton = tk.Button(self.activeMotorFrame, relief='groove',borderwidth=2, text='Retract Motor', command=retractMotor)
        self.initCameraButton = tk.Button(self.activeMotorFrame, relief='groove',borderwidth=2, text='Initialze Camera', command=initializeCamera)
        self.enableMotorButton = tk.Button(self.activeMotorFrame, relief='groove',borderwidth=2, text='Enable Motor', command=enableMotor)
        self.feedbackButton = tk.Button(self.activeMotorFrame, borderwidth=2, relief='groove', text='Apply Cyclic Feedback', command=toggleFeedback)
        
        self.initMotorButton.pack()
        self.retractMotorButton.pack()
        self.initCameraButton.pack()
        self.enableMotorButton.pack()
        self.feedbackButton.pack()

        self.activeFeedbackStretch = tk.IntVar(self.root, 15)
        self.activeFeedbackLabel = tk.Entry(self.activeMotorFrame)
        self.activeFeedbackLabel.pack()
        

        # LABELS

        self.cp
        self.frequncy
        self.distance
        self.enabled
        self.cameraInitialized
        self.motorHomed
        
        self.activeMotorPosition = tk.IntVar(self.activeMotorFrame, value=0)
        self.activePosition = tk.Label(self.activeMotorFrame)
        self.activePosition.pack()

        tk.Button(self.activeMotorFrame, relief='groove', text='-', command=adjustUp).pack(side=tk.RIGHT)
        tk.Button(self.activeMotorFrame, relief='groove', text='+', command=adjustDown).pack(side=tk.RIGHT)
        self.AllMotorStatus = tk.Frame(self.root, borderwidth=2, width=700, heigh=300)
        self.AllMotorStatus.place(x=50, y=400)
        self.motorStatus = []
        for i in range(4):
            self.motorStatus.append(tk.StringVar(self.AllMotorStatus, value='Idle'))
            self.subFrame = tk.Frame(self.AllMotorStatus, borderwidth=2, width=190, height=150)
            self.subFrame.place(x=5, y=(i*150+10))
            tk.Label(self.subFrame, textvariable=self.motorStatus[i]).place(x=5, y=5)
            
            def moveTo(motor=i):
                string='C'+str(motor)+','+str(self.WellLocations[motor])
                self.writeToSerial((string+'\n'))
                self.activeMotor = motor
                self.activeMotorLabel.configure(text='Active Motor: Row '+self.wellLabels[motor])
                
                
            self.moveToButton = tk.Button(self.subFrame, relief='groove', borderwidth=2, text='Move Camera\nto Row '+self.wellLabels[i], command=moveTo)
            self.moveToButton.place(x=20,y=20)
        pass
        
        self.activeMotorFrame.place(x=20,y=20)

    def writeToSerial(self, string):
        print(string)
        if self.ser != None:
            self.conn.write((string+'\n').encode())
        pass

    def mainloop(self):
        self.root.mainloop()


if __name__ == '__main__':
    root = tk.Tk()
    root.geometry("800x500")

    gui = CS3D_GUI(root)
    # root = tk.Tk()
    # gui = CS3D_GUI(root)
    # gui.update()
    # gui.run_update()
    gui.mainloop()