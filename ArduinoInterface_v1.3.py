import tkinter as tk
from serial import Serial, SerialException
import serial.tools.list_ports
import threading

class CS3D_GUI:
    def __init__(self, root, conn):
        self.root = root
        root.title('Cytostretcher 3D')

        self.conn = conn
        self.activeMotor = 0
        self.WellLocations = [0, 3875, 7775, 11700]
        self.motorFrames = []
        self.moveToButtons = []
        self.positions = []
        self.positionLabels = []
        self.SerialInput = tk.StringVar(self.root, value='1')
        for i in range(4):
            self.motorFrames.append(tk.Frame(self.root, background='white', border=5))
            self.positions.append(tk.StringVar(self.motorFrames[-1], value='0'))
            self.positionLabels.append(tk.Label(self.motorFrames[-1], background='white', foreground='black', textvariable=self.positions[-1]))
            self.moveToButtons.append(tk.Button(self.motorFrames[-1], text='Move Under Well '+str(i+1)))
            
            def moveTo(motor=i):
                string = 'C'+str(motor+1)+','+str(self.WellLocations[motor])
                self.conn.write((string+'\n').encode())
                self.activeMotor = motor + 1
                print(string)
            
            self.moveToButtons[-1].configure(command=moveTo)
            
            
            
            self.motorFrames[-1].pack()
            self.moveToButtons[-1].pack()
            self.positionLabels[-1].pack()
        

        self.EnableDisableButton = tk.Button(self.root, text="Enable/Disable Motor", command=self.EnableDisable)
        # self.greet_button = tk.Button(self.root, text="Greet", command=self.greet)
        self.EnableDisableButton.pack()

        self.textLabel = tk.Label(self.root, textvariable=self.SerialInput, fg='white')
        self.textLabel.pack()
        self.close_button = tk.Button(self.root, text="Close", command=root.destroy)
        self.close_button.pack()

    def EnableDisable(self):
        string = 'M'+str(self.activeMotor)
        self.conn.write((string+'\n').encode())
        print(string)

    def update(self):
        self.SerialInput.set(self.conn.readline().decode('utf-8')[:-2])
        # print(self.SerialInput)
        # self.textLabel.update()
        self.conn.flush()

        self.values = self.SerialInput.get().split(',')
        self.motorValues = []
        print(self.values)
        if len(self.values) > 1:
            
            self.t = self.values[0]
            for i in range(1,5):
                self.motorValues.append(self.values[i])
                self.positions[i-1].set(self.motorValues[-1].split('&')[2])
                self.positionLabels[i-1].update()
            # print(self.motorValues)

        self.root.after(33, self.update)
        
    def mainloop(self): # blocking
        self.root.mainloop()

    def run_update(self):
        self.thr = threading.Thread(target=self.update())
        self.thr.start() # run update function in background

serialports = serial.tools.list_ports.comports()

class EstablishConnection:
    def __init__(self, portlist):
        self.portlist = portlist
        self.portNames, self.descs, self.hwids = [],[],[]

        for port, desc, hwid in sorted(serialports):
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
                    k = k+1
                    continue
        
        if type(self.conn) == list:
            print('No Serial port found.')
            self.ActiveSerial = False

ser = EstablishConnection(serialports)
if ser.ActiveSerial == True:
    root = tk.Tk()
    gui = CS3D_GUI(root, EstablishConnection(serialports).conn)

# gui.update()
gui.run_update()
gui.mainloop()
