from serial import Serial, SerialException
import tkinter as tk
import threading
from time import time
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import list_ports_osx
import serial.tools.list_ports

ports = serial.tools.list_ports.comports()

portNames = []
descs = []
hwids = []
for port, desc, hwid in sorted(ports):
    portNames.append(port)
    descs.append(desc)
    hwids.append(hwid)
cameraLocations = [0, 3875, 7775, 11700]

print(descs)
def update():
    global ser
    global response
    
    # response = ''
    response = ser.readline().decode('utf-8')
    ser.flush()

    # print('got something')
        # while ('\n' not in response):
        #     response += ser.read().decode('utf-8')
        # # response = ser.readline().decode('utf-8')
    # ser.flushInput()
    # print(response[:-1])
    if response != '':
        try:
            motors = response.split(',')
            if len(motors) == 5:
                for motor in motors[:4]:
                    vals = motor.split('&')
                    id = int(vals[0]) - 1
                    t[id].set(vals[1])
                    positions[id].set(vals[2])
                    distances[id].set(vals[3])
                    freqs[id].set(vals[4])
                    if vals[5] == '0':    en[id].set('LOW')
                    else:               en[id].set('HIGH')
                    
                    if vals[6] == '0':    man[id].set('LOW')
                    else:               man[id].set('HIGH')
            cameraWell = motors[4].split('&')[0]
            cameraPosition = int(motors[4].split('&')[2][:-2])
            if cameraWell == str(-1):
                # print('Camera Moving')
                labelvar.set('Moving')
                label.update()
            else:
                # print(cameraWell + ': ' + str(cameraPosition))
                labelvar.set('Well: '+cameraWell)
                label.update()
        except:
            pass 

        
        # printToScreen(t,positions)

        # print(response[:-1])
    
    window.after(1, update)

def printToScreen(array1, array2):
    for i in range(len(array1)):
        print("%7s" % array1[i].get(), end=',')
        print("%3s" % array2[i].get(), end=' ')
    print(' ')

window = tk.Tk()

def sendCommand():
    if commandvar.get()[0] == 'C':
        ser.write((commandvar.get()+','+str(cameraLocations[int(commandvar.get()[1])-1]) + '\n').encode())
    else:
        ser.write((commandvar.get()+'\n').encode())
cameraWell = 1
commandvar = tk.StringVar(window, '')
labelvar = tk.StringVar(window, '')
positions = []
t = []
distances = []
freqs = []
en = []
man = []
for i in range(4):
    positions.append(tk.StringVar(window, str(i)))
    t.append(tk.StringVar(window, str(i)))
    distances.append(tk.StringVar(window, str(i)))
    freqs.append(tk.StringVar(window, str(i)))
    en.append(tk.StringVar(window, str(i)))
    man.append(tk.StringVar(window, str(i)))

response = ''
label = tk.Label(window, textvariable = labelvar, bg = 'white')
commandEntry = tk.Entry(window, textvariable = commandvar)
commandButton = tk.Button(window, text = 'Send to Motors', command=sendCommand)
label.place(x=0, y=0)
commandEntry.place(x=0,y=25)
commandButton.place(x=0,y=50)
for i in range(len(portNames)):
    if 'OpenMV' not in descs[i]:
        print(descs[i])
        ser = Serial(portNames[i], baudrate=9600, timeout=1)

ser.close()
ser.open()
ser.readline().decode('utf-8')
thr = threading.Thread(target=update())
thr.start() # run update function in background

window.mainloop()
ser.close()



########################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################################


from serial import Serial, SerialException
import tkinter as tk
import threading
from time import time
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import list_ports_osx
import serial.tools.list_ports



def toggleEnableDisable():
    print(currentwell.get())
    well = currentwell.get()
    print(textlabel[well-1].get()[9:])
    if textlabel[well-1].get()[9:] == 'Enabled':
        textlabel[well-1].set('Motor '+str(well)+': Disabled')
        frames[well-1].configure(background='blue')
        moveToWellButtons[well-1].configure(relief='raised')
        frames[well-1]
    else:
        textlabel[well-1].set('Motor '+str(well)+': Enabled')
        frames[well-1].configure(background='green')
        moveToWellButtons[well-1].configure(relief='sunken')
        

def moveToWell(well):
    # print(well)
    currentwell.set(well)
    wellLabel.set('Well '+str(currentwell.get()))
    print(currentwell.get())
    moveToWellButtons[well-1].configure(state=tk.DISABLED)
    otherindices = [i for i in range(4)]
    otherindices.pop(well-1)
    print(otherindices)
    for i in otherindices:
        moveToWellButtons[i].configure(state=tk.NORMAL)
    # change parameters to current well's parameters
    pass

def sendManualDistance():
    print(str(currentwell.get())+': '+manualPosition.get())
    pass


ports = serial.tools.list_ports.comports()

portNames, descs, hwids = [],[],[]
for port, desc, hwid in sorted(ports):
    portNames.append(port)
    descs.append(desc)
    hwids.append(hwid)
cameraLocations = [0, 3875, 7775, 11700]

ser = []
ActiveSerial = True
for port in portNames:
    if 'Bluetooth' in portNames:
        continue
    elif 'OpenMV' in portNames:
        continue
    else:  
        try:
            ser = Serial(port, baudrate=9600, timeout=1)
            ActiveSerial = True
        except:
            continue

a = type(ser)
if type(ser) == list:
    print('No Serial port found.')
    ActiveSerial = False

def update():
    global ser
    global response
    
    # response = ''
    response = ser.readline().decode('utf-8')
    ser.flush()

    motor = []
    wellnum, t_well, pos, dist, freq, enablestate, manualoverride = [1,2,3,4],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]
    t_well = [0,0,0,0]
    if len(response) != 0:
        # print(response[:-2])
        splitresponses = response[:-2].split(',')
        print(splitresponses)
        if type(splitresponses) == list:
            t = int(splitresponses[0])
            print(t)
            for i in range(4):
                print(splitresponses[i+1])
                motor.append(splitresponses[i+1].split('&'))
                print(motor[i])
                (wellnum[i], t_well[i], pos[i], dist[i], freq[i], enablestate[i], manualoverride[i]) = tuple(splitresponses[i+1].split('&'))
                positions[i].set(pos[i])
            print(wellnum)
            print(t_well)
            print(dist)
            print(freq)
            print(enablestate)
            print(manualoverride)
    root.after(1, update)
    pass


def sendCommand():
    if commandvar.get()[0] == 'C':
        ser.write((commandvar.get()+','+str(cameraLocations[int(commandvar.get()[1])-1]) + '\n').encode())
    else:
        ser.write((commandvar.get()+'\n').encode())


class CS3D_GUI:
    def __init__(self, root):
        self.root = root
        root.title('Cytostretcher 3D')


        return

root = tk.Tk()
root.configure(background='white')
currentwell = tk.IntVar(root, value=1)
manualPosition = tk.StringVar(root, value='0')
wellLabel = tk.StringVar(root, value='Well '+str(currentwell.get()))

frames = [tk.Frame(root, bg='white',  height=100, width=100, background='white') for i in range(4)]
textlabel = [tk.StringVar(frames[i], 'Motor '+str(i+1)+': Disabled') for i in range(4)]
labels = [tk.Label(frames[i], textvariable=textlabel[i], background='white', fg='black') for i in range(4)]
positions = [tk.StringVar(frames[i], value='0') for i in range(4)]
positionsLabel = [tk.Label(frames[i], textvariable=positions[i], bg='white', fg='black') for i in range(4)]
moveToWellButtons = [tk.Button(frames[0+0], text='Move to Well '+str(1+0), command=lambda: moveToWell(1+0), bg='white'),
                     tk.Button(frames[0+1], text='Move to Well '+str(1+1), command=lambda: moveToWell(1+1), bg='white'),
                     tk.Button(frames[0+2], text='Move to Well '+str(1+2), command=lambda: moveToWell(1+2), bg='white'),
                     tk.Button(frames[0+3], text='Move to Well '+str(1+3), command=lambda: moveToWell(1+3), bg='white')]

button = tk.Button(root, text='Enable/Disable Motor', command=toggleEnableDisable, background='white')
button.configure(bg='white')
manualSlide = tk.Scale(root, from_=0, to=1000, variable = manualPosition, orient='vertical', length=200, command=lambda val: sendManualDistance(), background='white')
currentwellLabel = tk.Label(root, textvariable=wellLabel, background='white', fg='black')

commandvar = tk.StringVar(root, '')
commandEntry = tk.Entry(root, textvariable = commandvar)
commandEntry.bind('<Return>', func=sendCommand)
# commandButton = tk.Button(root, text = 'Send to Motors', command=sendCommand)

commandEntry.pack()
for i in range(4):
    frames[i].pack()
    moveToWellButtons[i].pack()
    labels[i].pack()
    positionsLabel[i].pack()

currentwellLabel.pack()
button.pack()
manualSlide.pack()
thr = threading.Thread(target=update())
thr.start() # run update function in background
root.mainloop()