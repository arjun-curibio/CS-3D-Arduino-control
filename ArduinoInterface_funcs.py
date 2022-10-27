from asyncio import WriteTransport
from serial import Serial
import tkinter as tk
import threading
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import numpy as np
import json
import serial.tools.list_ports

class CS3D_GUI:
    def __init__(self, root, ser=None):
        self.root = root
        self.root.configure(background='white')
        root.title('Cytostretcher 3D')

        self.ser = ser
        if self.ser != None:
            self.conn = ser.conn
        self.t = 0
        self.activeMotor = 1
        self.WellLocations = [0, 3875, 7775, 11700]
        self.WellLocations = [0, 7775, 15550, 23325]
        self.wellLabels = ['D','C','B','A']
        # self.WellLocations = [0, 7775, 15550, 60000]
        self.motorFrames = []
        self.moveToButtons, self.retractButton, self.initMotorButton = [], [], []
        self.positions, self.positionLabels = [], []
        self.freqs, self.freqLabel = [], []
        self.dists, self.distLabel = [], []
        self.dist_num = [0,0,0,0]
        self.enable, self.enableLabel, self.motorEnableState = [], [], [0,0,0,0]
        self.passive_len = [0,0,0,0]
        self.desiredStretch = tk.StringVar(self.root, '20')
        self.positionHistory = []
        self.stretchHistory = []
        self.stretch = 0
        self.foundMax = 0
        self.maxs = ""
        self.cameraUnderWell = 0
        self.t_camera = 0
        self.SerialInput = tk.StringVar(self.root, value='1')
        self.output = tk.StringVar(self.root, value='')

        self.saveDataFlag = False
        self.applyFeedback = "Cyclic"
        self.feedbackActiveFlag = [False,False,False,False]
        self.counter = 0
        self.almostCounter = 0
        self.reachedDesiredStretch = [False, False, False, False]

        self.motorInited = [False, False, False, False]

        
        
        for i in range(4):
            self.motorFrames.append(tk.Frame(self.root, background='white', borderwidth=2, relief='groove', width=500, height=100))
            
            self.positions.append(tk.StringVar(self.motorFrames[-1], value='Position: 0 steps'))
            self.positionLabels.append(tk.Label(self.motorFrames[-1], background='white', foreground='black', textvariable=self.positions[-1]))
            
            self.moveToButtons.append(tk.Button(self.motorFrames[-1], relief='groove', borderwidth=2, text='Move Camera\nto Row '+self.wellLabels[i]))
            self.retractButton.append(tk.Button(self.motorFrames[-1], relief='groove', borderwidth=2, text='Retract Motor'))

            self.dists.append(tk.StringVar(self.motorFrames[-1], value='Distance: 20 steps'))
            self.distLabel.append(tk.Label(self.motorFrames[-1], textvariable=self.dists[i], foreground='black', background='white'))
            
            self.freqs.append(tk.StringVar(self.motorFrames[-1], value='Frequency: 1 Hz'))
            self.freqLabel.append(tk.Label(self.motorFrames[-1], textvariable=self.freqs[i], foreground='black', background='white'))
            
            self.enable.append(tk.StringVar(self.motorFrames[-1], value='Disabled'))
            self.enableLabel.append(tk.Label(self.motorFrames[-1], textvariable=self.enable[i], foreground='black', background='white'))
            
            def moveTo(motor=i):
                string = 'C'+str(motor)+','+str(self.WellLocations[motor])
                self.writeToSerial((string+'\n'))
                # self.conn.write((string+'\n').encode())
                self.motorFrames[int(self.cameraUnderWell)].configure(borderwidth=2)
                self.cameraUnderWell = motor
                self.motorFrames[int(self.cameraUnderWell)].configure(borderwidth=7)
                self.counter = 0
                self.almostCounter = 0
            def retractMotor(motor = i):
                string = 'X'+str(motor)
                self.writeToSerial(string+'\n')
            
            self.moveToButtons[-1].configure(command=moveTo)
            self.retractButton[-1].configure(command=retractMotor)

            self.motorFrames[-1].place(bordermode=tk.OUTSIDE, height=100, width=300, x=0, y=i*105)
            self.moveToButtons[-1].place(relwidth=0.3, relheight=0.5, relx=0.7, rely=0.05)
            self.retractButton[-1].place(relwidth=0.3, relheight=0.2, relx=0.7, rely=0.85)
            self.positionLabels[-1].place(relx=0.1, rely=0)
            self.freqLabel[-1].place(relx=0.1, rely=0.2)
            self.distLabel[-1].place(relx=0.1, rely=0.4)
            self.enableLabel[-1].place(relx=0.0, rely=0.6)
    
        self.resetCameraButton = tk.Button(self.root, borderwidth=2, relief='groove', text='Reset Camera\nPostion', command=self.resetCamera)
        self.resetCameraButton.place(x=100, y=430, width=100)
        self.retractAllButton = tk.Button(self.root, borderwidth=2, relief='groove', text='Retract All\nMotors', command=self.retractAllMotors)
        self.retractAllButton.place(x=200, y=430, width=100)
        
        self.initMotor = tk.Button(self.root, borderwidth=2, relief = 'groove', text='Home Motor', command=self.initializeMotor)
        self.initMotor.place(relx=0.5, rely=0.65)
        self.AdjUp  = tk.Button(self.root, relief='groove', borderwidth=2, text='-', command=lambda: self.sendAdj(0))
        self.AdjDown = tk.Button(self.root, relief='groove', borderwidth=2, text='+', command=lambda: self.sendAdj(1))
        self.AdjUp.place(relx=0.5-0.025, rely=0.65)
        self.AdjDown.place(relx=0.5+0.1, rely=0.65)
        self.InitCamera = tk.Button(self.root, borderwidth=2, relief='groove', text='INIT Camera', command=self.INITCAMERA)
        self.InitCamera.place(relx=0.5, rely=0.7)
        self.Output = tk.Entry(self.root, textvariable=self.output)
        self.Output.bind('<Return>', func=lambda val: self.sendOutput())
        self.Output.pack()

        self.EnableDisableButton = tk.Button(self.root, borderwidth=2, relief='groove', text="Enable/Disable Motor", command=self.EnableDisable)
        self.ResetButton = tk.Button(self.root, borderwidth=2, relief='groove', text='Zero Motor Position', command=self.ResetMotor)
        self.feedbackButton = tk.Button(self.root, text='Enable Feedback', borderwidth=2, relief='groove', command=self.toggleFeedback)
        
        self.EnableDisableButton.place(relx=0.5,rely=0.6)
        # self.ResetButton.pack()
        self.feedbackButton.place(relx=0.6, rely=0.55)
        self.desiredStretchEntry = tk.Entry(self.root, relief='groove',borderwidth=2, textvariable=self.desiredStretch)
        self.desiredStretchEntry.place(relx=0.55, rely=0.55, width=20)
        tk.Label(self.root, text="Stretch:").place(relx=0.49, rely=0.55)
        tk.Label(self.root, text="%").place(relx=0.575, rely=0.55, width=10)
        self.textLabel = tk.Label(self.root, textvariable=self.SerialInput, fg='black')
        # self.textLabel.pack()
        self.close_button = tk.Button(self.root, relief='groove', borderwidth=2, text="Close", command=root.destroy)
        self.close_button.place(x=700,y=400,width=100,height=50)

        self.saveButton = tk.Button(self.root, relief='groove', borderwidth=2, text="Save Data", command=self.saveData)
        self.saveButton.pack()
        self.saveLabel = tk.Label(self.root, text='')
        self.saveLabel.pack()
        self.motorFrames[self.cameraUnderWell].configure(borderwidth=7)

        magic = '0'
        self.t0 = 0
        # while magic != '-33' and magic != -1:
        #     string, magic = self.readFromSerial()
        #     self.root.after(1)
        #     self.t0 = int(string.split(';')[1])

        # self.fig = Figure(figsize=(2,2))
        # self.ax = self.fig.add_subplot(111)
        # self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        # self.canvas.draw()
        # self.canvas.get_tk_widget().place(relx=0.45, rely=0.6, relwidth=0.3, relheight=0.4)

    def resetCamera(self):
        self.writeToSerial('V'+'\n')
    
    def initializeMotor(self):
        string = "MOTORINIT&" + str(self.cameraUnderWell)
        self.writeToSerial("INIT"+'\n')
        self.root.after(1000)
        self.writeToSerial(string+'\n')

    def retractAllMotors(self):
        for i in range(4):
            self.writeToSerial('X'+str(i)+'\n')
            self.root.after(1)
        
    def readFromSerial(self):
        string = ''
        magic = -1
        if self.ser != None:
            if self.conn != []:
                string = self.conn.readline().decode('utf-8')[:-2]
                magic = string.split(',')[0]
                self.conn.flushInput()
        
        return string, magic
    def writeToSerial(self, string):
        if self.ser != None:
            self.conn.write(string.encode())
        print((str(self.t)+': '+string))
    def toggleFeedback(self):
        if self.feedbackActiveFlag[self.cameraUnderWell] == True:
            self.feedbackActiveFlag[self.cameraUnderWell] = False
            self.feedbackButton.configure(borderwidth=2)
        else:
            self.feedbackActiveFlag[self.cameraUnderWell] = True
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
    def plot(self):
        fig = Figure(figsize= (5,5), dpi=100)

        self.ax.clear()
        self.ax.plot(self.positionHistory, self.stretchHistory)
        pass
    def ResetMotor(self):
        string = 'R'+str(self.cameraUnderWell)
        self.writeToSerial(string+'\n')
        self.pos.set(0)
        self.Slider.update()
    def sendAdj(self, adj):
        string = 'A'+str(self.cameraUnderWell)+','+str(adj)
        self.writeToSerial(string+'\n')
    def sendPosition(self):
        string = 'O'+str(self.cameraUnderWell)+','+str(self.pos.get())
        self.writeToSerial(string+'\n')
    def sendOutput(self):
        self.writeToSerial(self.output.get()+'\n')
        self.output.set('')
        self.Output.update()
    def EnableDisable(self):
        if self.motorEnableState[self.cameraUnderWell] == 1 and self.motorInited[self.cameraUnderWell] == True:
            string = 'M'+str(self.cameraUnderWell)
            self.writeToSerial(string+'\n')
        elif self.motorEnableState[self.cameraUnderWell] == 0:
            string = 'M'+str(self.cameraUnderWell)
            self.writeToSerial(string+'\n')
        pass

    def update(self):
        if self.ser == None:
            string = '1'
        else:
            string, magic = self.readFromSerial()
            # print(self.feedbackActiveFlag)
            self.values = string.split(';')
            self.motorValues = []
            magic = self.values.pop(0)
            if magic == 'HELPER':
                pass
            elif magic == '-33':
                self.t = int(self.values.pop(0))
                self.t_camera, self.cameraUnderWell, self.MotorInitWell, self.recievedMotorInitHandshake, self.stretch, self.foundMax, self.max_stretch = tuple(self.values.pop(0).split('&'))
                # print(self.foundMax)
                self.cameraUnderWell = int(self.cameraUnderWell)
                for i in range(4):
                    motorID, motorT, position, dist, freq, passive_len, MotorInited, CameraInited, enableMotorState, beginMotorInitFlag = tuple(self.values[0].split('&'))
                    self.motorInited[i] = bool(int(MotorInited))
                    self.values.pop(0)
                    if self.feedbackActiveFlag[self.cameraUnderWell] == True and self.cameraUnderWell == i:
                        self.positions[i].set('Stretch: '+str(round(float(self.stretch),1))+' %')
                    else:
                        self.positions[i].set('Current position: '+position+' steps')
                    self.freqs[i].set('Frequency: '+freq+' Hz')
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
                self.cameraVals = self.values[-1].split('&')
                # print(self.stretchHistory)
                # print(self.stretch)
                # self.stretchHistory.append(float(self.stretch))
                self.positionHistory.append(float(self.positions[self.cameraUnderWell].get().split(': ')[1].split(' ')[0]))

                if len(self.positionHistory) > 100:
                #     self.stretchHistory.pop(0)
                    self.positionHistory.pop(0)

                # self.ax.clear()
                # self.ax.plot(self.positionHistory, self.stretchHistory)
                # self.canvas.draw()

                if self.feedbackActiveFlag[self.cameraUnderWell] == True:
                    self.applyFeedbackFunc()
                else:
                    pass
                    # val = "D"+str(self.cameraUnderWell)+','+str(self.dist_num[self.cameraUnderWell])
                    # print(val)

                
            # print([self.positions[i].get() for i in range(4)])
            if self.saveDataFlag == True:
                with open('samplefile.txt', 'a') as f:
                    f.write("{},{},{}\n".format(self.t, self.positions[int(self.cameraUnderWell)].get()[10:-5], self.stretch))
            
            # print("{}: {}".format(len(self.stretchHistory), self.stretchHistory))
            # print("{}: {}".format(len(self.positionHistory), self.positionHistory))
            # print(string)
            # print("{}, {}".format(self.positions[self.cameraUnderWell-1].get(),self.stretch))
            # print(self.positions[0].get().split(': ')[1].split(' ')[0])
            # print(self.cameraUnderWell)
            # print(string)
        if self.conn != []:
            self.conn.flush()
        self.root.after(1, self.update)
    
    def applyFeedbackFunc(self):
        if self.foundMax=='1' and self.cameraVals[0] != '-1' and self.motorEnableState[self.cameraUnderWell] == 0:
            # print(str(self.t)+':', end='')
            self.max_stretch_aslist = json.loads(self.max_stretch)[-1]
            
            max_stretch_active = round(100*np.mean(self.max_stretch_aslist)/self.passive_len[self.cameraUnderWell],1)
            # print(np.mean(self.max_stretch_aslist))
            
            diff = max_stretch_active - int(self.desiredStretch.get())
            self.reachedDesiredStretch[self.cameraUnderWell] = False
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
                    self.reachedDesiredStretch[self.cameraUnderWell] = True
            
            new_dist = -1*np.sign(diff)*new_dist
            val = "D"+str(self.cameraUnderWell)+','+str(int(self.dist_num[self.cameraUnderWell]+new_dist))
            print(val)
            self.writeToSerial(val+'\n')
        pass
    def mainloop(self): # blocking
        self.root.mainloop()

    def run_update(self):
        self.thr = threading.Thread(target=self.update())
        self.thr.start() # run update function in background


class EstablishConnection:
    def __init__(self, portlist):
        self.portlist = portlist
        self.portNames, self.descs, self.hwids = [],[],[]

        for port, desc, hwid in sorted(portlist):
            self.portNames.append(port)
            self.descs.append(desc)
            self.hwids.append(hwid)
        
        self.conn = []
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
