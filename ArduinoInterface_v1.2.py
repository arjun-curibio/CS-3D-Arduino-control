from serial import Serial, SerialException
import tkinter as tk
import threading
from time import time
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

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