from serial import Serial
import tkinter as tk
import threading

class CS3D_GUI:
    def __init__(self, root, conn=None):
        self.root = root
        root.title('Cytostretcher 3D')

        self.conn = conn
        self.t = 0
        self.activeMotor = 1
        # self.WellLocations = [0, 3875, 7775, 11700]
        self.WellLocations = [0, 7775, 15550, 23325]
        self.motorFrames = []
        self.moveToButtons = []
        self.positions = []
        self.positionLabels = []
        self.motorT = [0,0,0,0]
        self.freqs = []
        self.freqLabel = []
        self.dists = []
        self.distLabel = []
        self.motorEnable = [0,0,0,0]
        self.motorOverride = [0,0,0,0]
        self.positionHistory = []
        self.stretchHistory = []
        self.stretch = 0
        self.cameraUnderWell = 1
        self.t_camera = 0
        self.SerialInput = tk.StringVar(self.root, value='1')
        self.output = tk.StringVar(self.root, value='')
        self.pos = tk.IntVar(self.root, 0)

        self.saveDataFlag = False

        self.AdjUp  = tk.Button(self.root, text='-', command=lambda: self.sendAdj(0))
        self.AdjDown = tk.Button(self.root, text='+', command=lambda: self.sendAdj(1))
        self.Slider = tk.Scale(self.root, from_=-500, to=500, variable=self.pos, length=100, command=lambda val: self.sendPosition())
        self.Slider.pack(side=tk.RIGHT)
        self.AdjUp.pack(side=tk.RIGHT)
        self.AdjDown.pack(side=tk.RIGHT)
        
        for i in range(4):
            self.motorFrames.append(tk.Frame(self.root, background='white', borderwidth=2, relief='ridge', width=500, height=100))
            
            self.positions.append(tk.StringVar(self.motorFrames[-1], value='Position: 0 steps'))
            self.positionLabels.append(tk.Label(self.motorFrames[-1], background='white', foreground='black', textvariable=self.positions[-1]))
            
            self.moveToButtons.append(tk.Button(self.motorFrames[-1], text='Move Camera to Well '+str(i+1)))
            
            self.dists.append(tk.StringVar(self.motorFrames[-1], value='Distance: 20 steps'))
            self.distLabel.append(tk.Label(self.motorFrames[-1], textvariable=self.dists[i], foreground='black', background='white'))
            
            self.freqs.append(tk.StringVar(self.motorFrames[-1], value='Frequency: 1 Hz'))
            self.freqLabel.append(tk.Label(self.motorFrames[-1], textvariable=self.freqs[i], foreground='black', background='white'))
            
            def moveTo(motor=i):
                string = 'C'+str(motor+1)+','+str(self.WellLocations[motor])
                self.writeToSerial((string+'\n'))
                # self.conn.write((string+'\n').encode())
                self.cameraUnderWell = motor + 1
                print(string)
            
            self.moveToButtons[-1].configure(command=moveTo)
            
            
            
            self.motorFrames[-1].place(bordermode=tk.OUTSIDE, relheight=0.2, relwidth=0.8, relx=0, rely=i*0.2)
            self.moveToButtons[-1].place(relwidth=0.3, relheight=0.15, relx=0.7, rely=0.1)
            self.positionLabels[-1].pack()
            self.freqLabel[-1].pack()
            self.distLabel[-1].pack()
    
        
        self.InitCamera = tk.Button(self.root, text='INIT Camera', command=self.INITCAMERA)
        self.InitCamera.pack()
        self.Output = tk.Entry(self.root, textvariable=self.output)
        self.Output.bind('<Return>', func=lambda val: self.sendOutput())
        self.Output.pack()
        self.EnableDisableButton = tk.Button(self.root, text="Enable/Disable Motor", command=self.EnableDisable)
        self.ResetButton = tk.Button(self.root, text='Set Motor Position = 0', command=self.ResetMotor)
        # self.greet_button = tk.Button(self.root, text="Greet", command=self.greet)
        
        self.EnableDisableButton.pack()
        self.ResetButton.pack()
        self.textLabel = tk.Label(self.root, textvariable=self.SerialInput, fg='black')
        # self.textLabel.pack()
        self.close_button = tk.Button(self.root, text="Close", command=root.destroy)
        self.close_button.pack()

        self.saveButton = tk.Button(self.root, text="Save Data", command=self.saveData)
        self.saveButton.pack()
        self.saveLabel = tk.Label(self.root, text='')
        self.saveLabel.pack()

    def writeToSerial(self, string):
        if self.conn != None:
            self.conn.write(string.encode())
        
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
        self.writeToSerial(string+'\n')
    
    def plot(self):
        self.ax.clear()
        self.ax.plot(self.positionHistory, self.stretchHistory)
        pass

    def ResetMotor(self):
        string = 'R'+str(self.cameraUnderWell)
        self.writeToSerial(string+'\n')
        print(string)
        self.pos.set(0)
        self.Slider.update()


    def sendAdj(self, adj):
        string = 'A'+str(self.cameraUnderWell)+','+str(adj)
        if adj == 0:
            self.pos.set(self.pos.get() - 4)
            self.Slider.update()
        elif adj == 1:
            self.pos.set(self.pos.get() + 4)
            self.Slider.update()
        # self.pos = self.positions[self.activeMotor-1]
        # self.Slider.update()
        self.writeToSerial(string+'\n')
        print(string)

    def sendPosition(self):
        string = 'O'+str(self.cameraUnderWell)+','+str(self.pos.get())
        self.writeToSerial(string+'\n')
        print(string)

    def sendOutput(self):
        self.writeToSerial(self.output.get()+'\n')
        # print(self.output.get())
        self.output.set('')
        self.Output.update()
    
    def EnableDisable(self):
        string = 'M'+str(self.cameraUnderWell)
        self.writeToSerial(string+'\n')
        print(string)

    def update(self):
        if self.conn != None:
            string = self.conn.readline().decode('utf-8')[:-2]
            self.conn.flush()
        else:
            string = ''
        
        
        self.values = string.split(',')
        self.motorValues = []
        # print(self.values)
        if self.values[0] == '-33':   
            self.t = self.values[1]
            print(self.t, end=': ')
            n_before_motors = 3
            self.t_camera, self.cameraUnderWell, self.stretch = tuple(self.values[2].split('&'))
            # print(self.ImageProps)

            for i in range(n_before_motors,n_before_motors+4):
                self.motorValues.append(self.values[i])
                # print(self.motorValues[-1])
                motorID, motorT, position, dist, freq, motorEnable, motorOverride = tuple(self.motorValues[-1].split('&'))
                
                self.motorT[i-n_before_motors] = int(motorT)
                self.positions[i-n_before_motors].set('Position: '+position+' steps')
                self.freqs[i-n_before_motors].set('Frequency: '+freq+' Hz')
                self.dists[i-n_before_motors].set('Distance: '+dist+' steps')
                self.motorEnable[i-n_before_motors] = int(motorEnable)
                self.motorOverride[i-n_before_motors] = int(motorOverride)

                # print("{0},{1}".format(self.motorT[i-n_before_motors], self.positions[i-n_before_motors].get()), end=' // ')
                
                self.positionLabels[i-n_before_motors].update()
                # print(' // ', end=' ')
            # print(self.motorValues)
            # print(' ')
        # print([self.positions[i].get() for i in range(4)])
        if self.saveDataFlag == True:
            with open('samplefile.txt', 'a') as f:
                f.write("{},{},{}\n".format(self.t, self.positions[int(self.cameraUnderWell)-1].get()[10:-5], self.stretch))
        
        # print("{}: {}".format(len(self.stretchHistory), self.stretchHistory))
        # print("{}: {}".format(len(self.positionHistory), self.positionHistory))
        print(string)
        self.root.after(1, self.update)
    
    
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

