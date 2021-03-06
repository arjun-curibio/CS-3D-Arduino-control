from serial import Serial, SerialException
import tkinter as tk
import threading
from time import time

import serial.tools.list_ports
ports = serial.tools.list_ports.comports()
port = 'COM4'
ser = -1
cameraLocations = [2000, 33000, 63000, 95000]
comportEstablishedFlag = False
def EstablishConnection():   
    ports = serial.tools.list_ports.comports() # get all ports available
    
    global port
    global comportEstablishedFlag # flag whether port has been established
    global ser 
    ser = -1

    # if there are no ports, exit
    if len(ports) == 0:
        print("No serial ports found. Exiting.")
        return

    comportEstablishedFlag = False
    for port, desc, hwid in sorted(ports): # loop through each COM port
        
        print('Trying '+port)
        timeoutErrorFlag = False
        try: 
            ser = Serial(port, baudrate=9600, timeout=1) # try to establish serial connection with com port
            ser.flushInput()
            ser.write("Python Connection Established!\n".encode()) # send string
            response = ser.readline().decode('utf-8')
            if '\n' in response:
                inputRecieved = True
            # print(response)
            if not inputRecieved:
                comportEstablishedFlag = False
                print('No response. Trying another port, or exiting.')
                print('Incorrect or no response.  Trying another port, or exiting.')
                continue
            if inputRecieved and response != 'Arduino Connection Established!\n':
                comportEstablishedFlag = False
                print('Incorrect Response. Trying another port, or exiting.')
                continue
            elif response == 'Arduino Connection Established!\n':
                comportEstablishedFlag = True
                print('Connection Established ('+port+').')
                break
            
            if comportEstablishedFlag == True: # break out if flag
                return ser
        except: 
            continue
    
    return ser

def sendRecieveHandshake():
    global ser
    ser.flushInput()
    ser.write("Python handshake.\n".encode()) # send string`    `       `````````````   `
    top.after(1)
    response = ser.readline().decode('utf-8')
    if '\n' in response:
        inputRecieved = True
    print(response[:-2], end='')
    pass


ser = EstablishConnection()

# create GUI
top = tk.Tk()
top.geometry("800x350")

positions = [tk.StringVar(top, '0'),
             tk.StringVar(top, '0'),
             tk.StringVar(top, '0'),
             tk.StringVar(top, '0')]
cameraPosition = tk.StringVar(top, '1')

labels = [tk.Label(top, textvariable=positions[0], bg='white'),
          tk.Label(top, textvariable=positions[1], bg='white'),
          tk.Label(top, textvariable=positions[2], bg='white'),
          tk.Label(top, textvariable=positions[3], bg='white')]


distances = [tk.StringVar(top, '200'), #row A
             tk.StringVar(top, '200'),
             tk.StringVar(top, '200'),
             tk.StringVar(top, '200')   
            ]

frequencies = [tk.StringVar(top, '1'), 
                tk.StringVar(top, '1'), 
                tk.StringVar(top, '1'), 
                tk.StringVar(top, '1')]

sections = [tk.StringVar(top, '40'),
            tk.StringVar(top, '60'),
            tk.StringVar(top, '75')
            ]

m = [tk.StringVar(top, '0'),
     tk.StringVar(top, '0'),
     tk.StringVar(top, '0'),
     tk.StringVar(top, '0')
     ]

# update GUI based on serial input
t_handshake = time()
print()
print()
def update():
    global ser
    global t_handshake
    # if comportEstablishedFlag:
    if time() > t_handshake+10:
        # print()
        # print("Sent handshake... ", end='')
        sendRecieveHandshake()
        t_handshake=time()
    response = ''
    try:
        if ser.in_waiting > 0:
            # print('Input recieved: ', end='')
            response = ser.readline().decode('utf-8')
            # print(str(time())+':', end='')
            # print(response, end='                                                                     \033[F')
            # print(response, end='                                                    \r')
            ser.flushInput()
        else:
            pass
            
    except SerialException as e:
        response = 'Serial input error.'
    if ('\n' in response):
        lineRecieved = True

    #print(response[:10])
    # print(pos)
    
    if response[:6] == 'RESET:':
        value = response[6:-1]
        # print(value+'\n')
        m[int(value)].set(0)
        M[int(value)].set(0)
    if response[:10]== 'POSITIONS:': # only update if correct line comes back
        values = response[10:-2].split(';') # split based on delimiter
        print(values, end='                                           \r')
        
        if len(values) == 5: # only if the length of what comes back is 5
            for ii in range(4):
                positions[ii].set(float(values[ii].split(',')[1])) # set position value
                labels[ii].update() # update label
            cameraPosition.set(float(values[4]))
                
                
    
    #print(' ')  
    # ser.flushInput()
    top.after(1, update)

# threading.Thread(target=update()).start() # run update function in background

def toggleState(a='R'):
    # if comportEstablishedFlag:
    ser.write((a+'\n').encode())
    print('\n'+a)
    

def destroyAndClose():
    ser.write('Q\n'.encode())
    ser.close()
    top.destroy()

def setDistance(command):
    ser.write((command+distances[int(command[0])-1].get()+'\n').encode())
    

def setFrequency(command):
    ser.write((command+frequencies[int(command[0])-1].get()+'\n').encode())
    
def adjustPosition(motor, distance):
    command = motor+'M'+str(M[int(motor)].get()+int(distance))
    M[int(motor)-1].set(M[int(motor)-1].get()+int(distance))

    ser.write((command+'\n').encode())
    
def adjustWaveform():
    if int(sections[0].get()) < 10 and len(sections[0].get()) == 1:
        sections[0].set('0'+sections[0].get())
    if int(sections[1].get()) < 10 and len(sections[1].get()) == 1:
        sections[1].set('0'+sections[1].get())
    if int(sections[2].get()) < 10 and len(sections[2].get()) == 1:
        sections[2].set('0'+sections[2].get())
    ser.write(('S,'+sections[0].get()+','+sections[1].get()+','+sections[2].get()+'\n').encode())
    
def sendManualDistance(motor):
    ser.write( (motor+'M'+str(M[int(motor)-1].get())+'\n').encode() )
    #print((motor+'M'+str(M[int(motor)-1].get())+'\n'), end='')


x = 25
y = 75
sx = 125
sy = 25
w = 35

motorFrame = [  tk.Frame(top, bg='white'), tk.Frame(top, bg='white'), tk.Frame(top, bg='white'), tk.Frame(top, bg='white')]
W = [tk.Button(), tk.Button(), tk.Button(), tk.Button()]
F = [tk.Button(), tk.Button(), tk.Button(), tk.Button()]
D = [tk.Button(), tk.Button(), tk.Button(), tk.Button()]
M = [tk.Scale(), tk.Scale(), tk.Scale(), tk.Scale()]
C = [tk.Button(), tk.Button(), tk.Button(), tk.Button()]

Ap = [tk.Button(), tk.Button(), tk.Button(), tk.Button()]
Am = [tk.Button(), tk.Button(), tk.Button(), tk.Button()]
dEntry = [tk.Entry(), tk.Entry(), tk.Entry(), tk.Entry()]
fEntry = [tk.Entry(), tk.Entry(), tk.Entry(), tk.Entry()]

W[0] = tk.Button(motorFrame[0], text='Pause/\nPlay M'+str(0+1), command=lambda: toggleState(str(0+1)+'W'))
W[1] = tk.Button(motorFrame[1], text='Pause/\nPlay M'+str(1+1), command=lambda: toggleState(str(1+1)+'W'))
W[2] = tk.Button(motorFrame[2], text='Pause/\nPlay M'+str(2+1), command=lambda: toggleState(str(2+1)+'W'))
W[3] = tk.Button(motorFrame[3], text='Pause/\nPlay M'+str(3+1), command=lambda: toggleState(str(3+1)+'W'))

F[0] = tk.Button(motorFrame[0], text="Set M"+str(0+1)+' Frequency', command=lambda: setFrequency(str(0+1)+'F'))
F[1] = tk.Button(motorFrame[1], text="Set M"+str(1+1)+' Frequency', command=lambda: setFrequency(str(1+1)+'F'))
F[2] = tk.Button(motorFrame[2], text="Set M"+str(2+1)+' Frequency', command=lambda: setFrequency(str(2+1)+'F'))
F[3] = tk.Button(motorFrame[3], text="Set M"+str(3+1)+' Frequency', command=lambda: setFrequency(str(3+1)+'F'))

D[0] = tk.Button(motorFrame[0], text="Set M"+str(0+1)+' Distance', command=lambda: setDistance(str(0+1)+'D'))
D[1] = tk.Button(motorFrame[1], text="Set M"+str(1+1)+' Distance', command=lambda: setDistance(str(1+1)+'D'))
D[2] = tk.Button(motorFrame[2], text="Set M"+str(2+1)+' Distance', command=lambda: setDistance(str(2+1)+'D'))
D[3] = tk.Button(motorFrame[3], text="Set M"+str(3+1)+' Distance', command=lambda: setDistance(str(3+1)+'D'))

C[0] = tk.Button(top, text="Move Camera\nunder Motor "+str(0+1), command=lambda: toggleState('C'+str(cameraLocations[0])))
C[1] = tk.Button(top, text="Move Camera\nunder Motor "+str(1+1), command=lambda: toggleState('C'+str(cameraLocations[1])))
C[2] = tk.Button(top, text="Move Camera\nunder Motor "+str(2+1), command=lambda: toggleState('C'+str(cameraLocations[2])))
C[3] = tk.Button(top, text="Move Camera\nunder Motor "+str(3+1), command=lambda: toggleState('C'+str(cameraLocations[3])))

M[0] = tk.Scale(motorFrame[0], from_=-1000, to=1000, variable=m[0], orient='vertical', length=150, command=lambda val: sendManualDistance(str(0+1)))
M[1] = tk.Scale(motorFrame[1], from_=-1000, to=1000, variable=m[1], orient='vertical', length=150, command=lambda val: sendManualDistance(str(1+1)))
M[2] = tk.Scale(motorFrame[2], from_=-1000, to=1000, variable=m[2], orient='vertical', length=150, command=lambda val: sendManualDistance(str(2+1)))
M[3] = tk.Scale(motorFrame[3], from_=-1000, to=1000, variable=m[3], orient='vertical', length=150, command=lambda val: sendManualDistance(str(3+1)))

Ap[0] = tk.Button(motorFrame[0], text='+', command=lambda: adjustPosition(str(0+1), '1'))
Ap[1] = tk.Button(motorFrame[1], text='+', command=lambda: adjustPosition(str(1+1), '1'))
Ap[2] = tk.Button(motorFrame[2], text='+', command=lambda: adjustPosition(str(2+1), '1'))
Ap[3] = tk.Button(motorFrame[3], text='+', command=lambda: adjustPosition(str(3+1), '1'))

Am[0] = tk.Button(motorFrame[0], text='-', command=lambda: adjustPosition(str(0+1), '-1'))
Am[1] = tk.Button(motorFrame[1], text='-', command=lambda: adjustPosition(str(1+1), '-1'))
Am[2] = tk.Button(motorFrame[2], text='-', command=lambda: adjustPosition(str(2+1), '-1'))
Am[3] = tk.Button(motorFrame[3], text='-', command=lambda: adjustPosition(str(3+1), '-1'))

dEntry[0] = tk.Entry(motorFrame[0], textvariable=distances[0])
dEntry[1] = tk.Entry(motorFrame[1], textvariable=distances[1])
dEntry[2] = tk.Entry(motorFrame[2], textvariable=distances[2])
dEntry[3] = tk.Entry(motorFrame[3], textvariable=distances[3])

fEntry[0] = tk.Entry(motorFrame[0], textvariable=frequencies[0])
fEntry[1] = tk.Entry(motorFrame[1], textvariable=frequencies[1])
fEntry[2] = tk.Entry(motorFrame[2], textvariable=frequencies[2])
fEntry[3] = tk.Entry(motorFrame[3], textvariable=frequencies[3])

tk.Label(top, bg='white', textvariable=cameraPosition).place(x=750, y=300, width=100)

for ii in range(4):
    motorFrame[ii].place(x=10+(ii*190), y=10, width=175, height=150)
    M[ii].set(0), m[ii].set(0)
    M[ii].place(x=125, y=0)
    W[ii].place(x=80, y=90)
    # R[ii].place(x= 20, y=110)
    tk.Label(motorFrame[ii], bg='white', text='MOTOR '+str(ii+1)).pack(side='top')
    D[ii].place(x=10, y=20, width=100)
    F[ii].place(x=10, y=50, width=100)
    dEntry[ii].place(x=110, y=23, width=25), tk.Label(motorFrame[ii], bg='white', text='steps').place(x=140, y=23)
    fEntry[ii].place(x=110, y=53, width=25), tk.Label(motorFrame[ii], bg='white', text='Hz').place(x=140, y=53)
    tk.Label(motorFrame[ii], bg='white', text='Adj.').place(x=0, y=85, width=25)
    Am[ii].place(x=25, y=85, width=20)
    Ap[ii].place(x=50, y=85, width=20)
    tk.Label(top, text='Current position:', bg='white').place(x=10+(ii*190), y=165)

    C[ii].place(x=20+(ii*190), y=200)
    labels[ii].place(x=100+(ii*190), y=165, width=100)
        
E = tk.Button(top, text='PAUSE/UNPAUSE ALL MOTORS', command=lambda: toggleState('E'))
X = tk.Button(top, text='EMERGENCY MOTOR RETRACT', command=lambda: toggleState('X'))
SRISE = tk.Entry(top, textvariable=sections[0])
SHOLD = tk.Entry(top, textvariable=sections[1])
SFALL = tk.Entry(top, textvariable=sections[2])
S = tk.Button(top, text='UPDATE WAVEFORM', command=adjustWaveform)
CR = tk.Button(top, text='Reset\nCamera', command=lambda: toggleState('CR'))

# Q.place(x=250, y=325)
# RESET.place(x=250, y=275)
E.place(x=250, y=275)
X.place(x=250, y=300)

SRISE.place(x=200, y=250, width=35)
SHOLD.place(x=250, y=250, width=35)
SFALL.place(x=300, y=250, width=35)
S.place(x=350, y=250)
CR.place(x=700, y=200)
update()
# reader.start()
# top.update_idletasks()

# RESET ALL MOTORS BEFORE OPENING GUI
# for ii in range(4):
#     ser.write( (str(ii+1)+'M'+str(-5000)+'\n').encode() )
    
for ii in range(4):
    toggleState(str(ii+1)+'R')
    top.after(1)
    ser.write((str(ii+1)+'D'+distances[int(ii)-1].get()+'\n').encode())
    top.after(1)
    ser.write((str(ii+1)+'F'+frequencies[int(ii)-1].get()+'\n').encode())


toggleState('B')

top.mainloop()

toggleState('Q')
#toggleState('R')

#destroyAndClose()
