import json, sys, pyopenmv, pygame
from serial import Serial
from serial.tools.list_ports import comports
from time import time

class EstablishConnections:
    def __init__(self):
        self.connected_to_motors = False
        self.connected_to_camera=False
        self.conn = None

        self.ports = comports()
        [print("{}: {} ({})".format(port, desc, hwid)) for port, desc, hwid in self.ports]
        
    
    def connect_to_camera(self):
        for port, desc, hwid in comports():
            if 'OpenMV' in port or 'OpenMV' in desc or 'OpenMV' in hwid:
                try:
                    print('initing')
                    pyopenmv.init(port, baudrate=921600, timeout=0.010)
                    pyopenmv.set_timeout(1*2) # SD Cards can cause big hicups.
                    pyopenmv.stop_script() # stop any script thats working
                    pyopenmv.enable_fb(True) # enable frame buffer (NEEDS TO BE ENABLED TO GRAB FROM FRAME BUFFER)

                    try:
                        with open('D:/main.py', 'r') as f:
                            script=f.read()
                    except:
                        print('errored')
                        script = ""
                    
                    print(len(script))
                    pyopenmv.exec_script(script) # Execute the script that's located in D:/main.py
                    self.connected_to_camera = True
                    print("Connected to camera.")
                except:
                    self.connected_to_camera=False
                    print("Failed to connect to camera.")
                continue
            else:  
                continue
    def connect_to_motors(self):
        for port, desc, hwid in comports():
            if 'OpenMV' in port or 'OpenMV' in desc or 'OpenMV' in hwid:
                continue
            elif 'Bluetooth' in port or 'Bluetooth' in desc or 'Bluetooth' in hwid:
                continue
            else:  
                try:
                    self.conn = Serial(port, baudrate=2000000, timeout=1)
                    self.connected_to_motors = True
                    print('Connected to motors. ')
                    continue
                except:
                    self.conn = None
                    self.connected_to_motors = False
                    print('Tried to connect but failed.  Trying another or exiting.')
                    continue
        return self.conn
    
    def close(self):
        self.conn.close()


conn = EstablishConnections()
conn.connect_to_camera()
pygame.init()
screen = pygame.display.set_mode((640,480), pygame.DOUBLEBUF)
got_image = False
screen_font = pygame.font.SysFont("monospace", 33)
bottom_screen_font = pygame.font.SysFont("monospace", 14)

t0 = time()
t = t0
def get_frame_buffer():
    global t
    # print('getting_frame_buffer')
    frame_buffer = pyopenmv.fb_dump()
    if frame_buffer != None:
        frame = frame_buffer[2]
        size = (frame_buffer[0], frame_buffer[1])
        # create image from RGB888
        image = pygame.image.frombuffer(frame.flat[0:], size, 'RGB')

        # blit stuff
        screen.blit(image, (0, 0))
        t = time() - t0
        # self.screen.blit(self.font.render("FPS %.2f"%(fps), 1, (255, 0, 0)), (0, 0))            
        return image
    else:
        t - time() - t0
        return None
    
running = True
# print('adding rect')
rectangle = pygame.rect.Rect(176, 134, 17, 17)
rectangle_dragging = False
offset_x, offset_y = 0,0
make_a_rect = False
rect_making = False
x1, x2, y1, y2 = 0,0,0,0
def do_pygame_events():
    # print('do_events')
    global rectangle_dragging, running, offset_x, offset_y
    global x1,x2,y1,y2,make_a_rect, rect_making
    for event in pygame.event.get():
        print(event.type)
        if event.type == pygame.QUIT:
            running = False
        
        elif event.type == pygame.MOUSEBUTTONDOWN:
            print(event.button)
            
            if event.button == 3:
                make_a_rect = True
                if rect_making == False:
                    x1, y1 = event.pos
                    x2, y2 = event.pos
                    rect_making = True
                

        elif event.type == pygame.MOUSEBUTTONUP:
            
            if event.button == 3:
                rect_making = False
                make_a_rect = False
            # if event.button == 3:
            #     rect_making = False
            #     make_a_rect = False
            #     x2, y2 = event.pos

        elif event.type == pygame.MOUSEMOTION:
            if rect_making:
                x2, y2 = event.pos
            

while running:
    img = get_frame_buffer()
    do_pygame_events()
   
    if img is not None:
        print(t)
       
        # pygame.draw.rect(screen, (255,0,0), rectangle)
        if make_a_rect:
            top_left = (min(x1, x2), min(y1, y2))
            top_right = (max(x1, x2), min(y1, y2))
            bottom_left = (min(x1, x2), max(y1, y2))
            bottom_right = (max(x1, x2), max(y1, y2))
            
            color = (255, 255, 0)
            
            pygame.draw.line(screen, color, top_left, top_right, 2) # left
            pygame.draw.line(screen, color, top_left, bottom_left, 2) # bottom
            pygame.draw.line(screen, color, top_right, bottom_right, 2) # right
            pygame.draw.line(screen, color, bottom_left, bottom_right, 2) # top
            
            # pygame.draw.rect(screen, (255,255,255), pygame.rect.Rect(min(x1,x2), min(y1,y2), abs(x1-x2), abs(y1-y2)))
        
    pygame.display.flip() # update screen


            
        

