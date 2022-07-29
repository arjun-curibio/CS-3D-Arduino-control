import tkinter as tk
import serial.tools.list_ports
from ArduinoInterface_funcs import *


serialports = serial.tools.list_ports.comports()
ser = EstablishConnection(serialports)
if ser.ActiveSerial == True:
    root = tk.Tk()
    root.geometry("800x500")
    gui = CS3D_GUI(root, ser.conn)

# root = tk.Tk()

# gui = CS3D_GUI(root)
# gui.update()
gui.run_update()
gui.mainloop()


