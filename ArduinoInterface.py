import serial
import tkinter as tk
import threading

import serial.tools.list_ports
ports = serial.tools.list_ports.comports()

for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))

comport = "COM3"
ser = serial.Serial(comport, 9600, timeout=0)
print(ser.name)

top = tk.Tk()
top.geometry("800x350")

positions = [tk.StringVar(top, '0'),
             tk.StringVar(top, '0'),
             tk.StringVar(top, '0'),
             tk.StringVar(top, '0')]

labels = [tk.Label(top, textvariable=positions[0], bg='white'),
          tk.Label(top, textvariable=positions[1], bg='white'),
          tk.Label(top, textvariable=positions[2], bg='white'),
          tk.Label(top, textvariable=positions[3], bg='white')]

for ii in range(4):
    labels[ii].pack

distances = [tk.StringVar(top, '20'),
             tk.StringVar(top, '20'),
             tk.StringVar(top, '20'),
             tk.StringVar(top, '20')   
            ]

frequencies = [tk.StringVar(top, '1'), 
                tk.StringVar(top, '1'), 
                tk.StringVar(top, '1'), 
                tk.StringVar(top, '1')]

sections = [tk.StringVar(top, '40'),
            tk.StringVar(top, '60'),
            tk.StringVar(top, '75')
            ]
# class SerialReaderProtocolRaw(Protocol):
#     port=None

#     def connection_made(self, transport):
#         print("Connected, ready to recieve data...")
#     def data_recieved(self, data):
#         updateLabelData(data)

# def updateLabelData(data):
#     data = ser.readline().decode("utf-8")
#     label['text']=data
#     top.update_idletasks()



def update():
    response = ''
    numLines = 0
    while True:
        response += ser.read().decode("utf-8")
        if ('\n' in response):
            numLines = numLines + 1
        if (numLines >= 1):
            break
    
    pos = response.split(',')
    if len(pos) >= 4:
        for ii in range(4):
            positions[ii].set(pos[ii])
    for ii in range(4):     
        labels[ii].update()
        #print("Motor "+str(ii+1)+': '+ positions[ii].get()+',\t', end=' ')
        pass
        
    #print(' ')  
    # ser.flushInput()
    top.after(1, update)

threading.Thread(target=update()).start()

def toggleState(a='R'):
    ser.write((a+'\n').encode())
    #print((a+'\n'))
    

def destroyAndClose():
    ser.write('Q\n'.encode())
    ser.close()
    top.destroy()

def setDistance(command):
    ser.write((command+distances[int(command[0])-1].get()+'\n').encode())
    #print((command+frequencies[int(command[0])-1].get()))
    

def setFrequency(command):
    ser.write((command+frequencies[int(command[0])-1].get()+'\n').encode())
    #print((command+frequencies[int(command[0])-1].get()))
    
def adjustPosition(command):
    ser.write((command+'\n').encode())
    #print(command)

def adjustWaveform():
    if int(sections[0].get()) < 10 and len(sections[0].get()) == 1:
        sections[0].set('0'+sections[0].get())
    if int(sections[1].get()) < 10 and len(sections[1].get()) == 1:
        sections[1].set('0'+sections[1].get())
    if int(sections[2].get()) < 10 and len(sections[2].get()) == 1:
        sections[2].set('0'+sections[2].get())
    ser.write(('S,'+sections[0].get()+','+sections[1].get()+','+sections[2].get()+'\n').encode())
    #print('S,'+sections[0].get()+','+sections[1].get()+','+sections[2].get()+'\n')


x = 25
y = 75
sx = 125
sy = 25
w = 35

motorFrame = [  tk.Frame(top, bg='white'), tk.Frame(top, bg='white'), tk.Frame(top, bg='white'), tk.Frame(top, bg='white')]
W = [tk.Button(), tk.Button(), tk.Button(), tk.Button()]
W[0] = tk.Button(motorFrame[0], text='Pause/\nPlay M'+str(0+1), command=lambda: toggleState(str(0+1)+'W'))
W[1] = tk.Button(motorFrame[1], text='Pause/\nPlay M'+str(1+1), command=lambda: toggleState(str(1+1)+'W'))
W[2] = tk.Button(motorFrame[2], text='Pause/\nPlay M'+str(2+1), command=lambda: toggleState(str(2+1)+'W'))
W[3] = tk.Button(motorFrame[3], text='Pause/\nPlay M'+str(3+1), command=lambda: toggleState(str(3+1)+'W'))

R = [tk.Button(), tk.Button(), tk.Button(), tk.Button()]
R[0] = tk.Button(motorFrame[0], text="Reset M"+str(0+1), command=lambda: toggleState(str(0+1)+'R'))
R[1] = tk.Button(motorFrame[1], text="Reset M"+str(1+1), command=lambda: toggleState(str(1+1)+'R'))
R[2] = tk.Button(motorFrame[2], text="Reset M"+str(2+1), command=lambda: toggleState(str(2+1)+'R'))
R[3] = tk.Button(motorFrame[3], text="Reset M"+str(3+1), command=lambda: toggleState(str(3+1)+'R'))

F = [tk.Button(), tk.Button(), tk.Button(), tk.Button()]
F[0] = tk.Button(motorFrame[0], text="Set M"+str(0+1)+'Frequency', command=lambda: setFrequency(str(0+1)+'F'))
F[1] = tk.Button(motorFrame[1], text="Set M"+str(1+1)+'Frequency', command=lambda: setFrequency(str(1+1)+'F'))
F[2] = tk.Button(motorFrame[2], text="Set M"+str(2+1)+'Frequency', command=lambda: setFrequency(str(2+1)+'F'))
F[3] = tk.Button(motorFrame[3], text="Set M"+str(3+1)+'Frequency', command=lambda: setFrequency(str(3+1)+'F'))

D = [tk.Button(), tk.Button(), tk.Button(), tk.Button()]
D[0] = tk.Button(motorFrame[0], text="Set M"+str(0+1)+'Distance', command=lambda: setDistance(str(0+1)+'D'))
D[1] = tk.Button(motorFrame[1], text="Set M"+str(1+1)+'Distance', command=lambda: setDistance(str(1+1)+'D'))
D[2] = tk.Button(motorFrame[2], text="Set M"+str(2+1)+'Distance', command=lambda: setDistance(str(2+1)+'D'))
D[3] = tk.Button(motorFrame[3], text="Set M"+str(3+1)+'Distance', command=lambda: setDistance(str(3+1)+'D'))

Ap = [tk.Button(), tk.Button(), tk.Button(), tk.Button()]
Ap[0] = tk.Button(motorFrame[0], text='+', command=lambda: adjustPosition(str(0+1)+'A1'))
Ap[1] = tk.Button(motorFrame[1], text='+', command=lambda: adjustPosition(str(1+1)+'A1'))
Ap[2] = tk.Button(motorFrame[2], text='+', command=lambda: adjustPosition(str(2+1)+'A1'))
Ap[3] = tk.Button(motorFrame[3], text='+', command=lambda: adjustPosition(str(3+1)+'A1'))

Am = [tk.Button(), tk.Button(), tk.Button(), tk.Button()]
Am[0] = tk.Button(motorFrame[0], text='-', command=lambda: adjustPosition(str(0+1)+'A0'))
Am[1] = tk.Button(motorFrame[1], text='-', command=lambda: adjustPosition(str(1+1)+'A0'))
Am[2] = tk.Button(motorFrame[2], text='-', command=lambda: adjustPosition(str(2+1)+'A0'))
Am[3] = tk.Button(motorFrame[3], text='-', command=lambda: adjustPosition(str(3+1)+'A0'))

dEntry = [tk.Entry(), tk.Entry(), tk.Entry(), tk.Entry()]
fEntry = [tk.Entry(), tk.Entry(), tk.Entry(), tk.Entry()]
dEntry[0] = tk.Entry(motorFrame[0], textvariable=distances[0])
dEntry[1] = tk.Entry(motorFrame[1], textvariable=distances[1])
dEntry[2] = tk.Entry(motorFrame[2], textvariable=distances[2])
dEntry[3] = tk.Entry(motorFrame[3], textvariable=distances[3])

fEntry[0] = tk.Entry(motorFrame[0], textvariable=frequencies[0])
fEntry[1] = tk.Entry(motorFrame[1], textvariable=frequencies[1])
fEntry[2] = tk.Entry(motorFrame[2], textvariable=frequencies[2])
fEntry[3] = tk.Entry(motorFrame[3], textvariable=frequencies[3])

for ii in range(4):
    motorFrame[ii].place(x=10+(ii*190), y=10, width=175, height=150)
    W[ii].place(x=100, y=90)
    R[ii].place(x= 20, y=110)
    tk.Label(motorFrame[ii], bg='white', text='MOTOR '+str(ii+1)).pack(side='top')
    D[ii].place(x=10, y=20, width=100)
    F[ii].place(x=10, y=50, width=100)
    dEntry[ii].place(x=110, y=23, width=25), tk.Label(motorFrame[ii], bg='white', text='steps').place(x=140, y=23)
    fEntry[ii].place(x=110, y=53, width=25), tk.Label(motorFrame[ii], bg='white', text='Hz').place(x=140, y=53)
    tk.Label(motorFrame[ii], bg='white', text='Adj.').place(x=0, y=85, width=25)
    Am[ii].place(x=25, y=85, width=20)
    Ap[ii].place(x=50, y=85, width=20)
    tk.Label(top, text='Current position:', bg='white').place(x=10+(ii*190), y=165)
    labels[ii].place(x=100+(ii*190), y=165, width=25)
        
RESET = tk.Button(top, text="RESET ALL MOTORS", command=lambda: toggleState('R'))
E = tk.Button(top, text='PAUSE/UNPAUSE ALL MOTORS', command=lambda: toggleState('E'))
Q = tk.Button(top, text='QUIT PROGRAM', command=destroyAndClose)

SRISE = tk.Entry(top, textvariable=sections[0])
SHOLD = tk.Entry(top, textvariable=sections[1])
SFALL = tk.Entry(top, textvariable=sections[2])
S = tk.Button(top, text='UPDATE WAVEFORM', command=adjustWaveform)

Q.place(x=250, y=300)
RESET.place(x=250, y=275)
E.place(x=250, y=250)

SRISE.place(x=200, y=200, width=35)
SHOLD.place(x=250, y=200, width=35)
SFALL.place(x=300, y=200, width=35)
S.place(x=350, y=200)
# update()
# reader.start()
# top.update_idletasks()
top.mainloop()