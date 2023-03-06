import cv2
from cv2 import COLOR_RGBA2BGR
import pygame
import sys
import numpy

if __name__ == '__main__':

    # This algorithm is using pygame library to generate the game sprite animation,
    # For each new frame, the current display (SCREEN) is converted to a string buffer and pushed
    # into a python list (VIDEO), all the data are uncompressed and the memory can be filled very quickly
    # especially with FPS > 30 
    # After pressing ESCAPE, all the recorded frames are converted to pygame surfaces before being
    # compressed into an AVI file
    # ** NOTE: No sound will be added to the final AVI file.
    # If you want to contribute and add some code to generate an AVI file with sound stream;
    # feel free to post your own version.

    SCREENRECT = pygame.Rect(0, 0, 800, 1024)
    pygame.display.init()
    SCREEN = pygame.display.set_mode(SCREENRECT.size, pygame.HWACCEL, 32)
    pygame.init()
    pygame.mixer.pre_init(44100, 16, 2, 4095)

    STOP_GAME = False

    RECORDING = True   # Display recording allowed True | False
    VIDEO = []         # string buffer

    clock = pygame.time.Clock()
    FRAME = 0
    
    

    while not STOP_GAME:

        pygame.event.pump()

        keys = pygame.key.get_pressed()

        if keys[pygame.K_ESCAPE]:
            STOP_GAME = True

        # *** UPDATE AND DRAW YOUR SPRITES HERE ***

        # Cap the speed at 30 FPS
        TIME_PASSED_SECONDS = clock.tick(30)

        pygame.display.flip()
      
        # Creates uncompress string that can be transferred with the 'fromstring' method in other Python imaging packages.
        # Some Python image packages prefer their images in bottom-to-top format (PyOpenGL for example).
        # If you pass True for the flipped argument, the string buffer will be vertically flipped.
        # The format argument is a string of one of the following values. Note that only 8-bit Surfaces can
        # use the "P" format. The other formats will work for any Surface.
        # Also note that other Python image packages support more formats than pygame.
        # P, 8-bit palettized Surfaces
        # RGB, 24-bit image
        # RGBX, 32-bit image with unused space
        # RGBA, 32-bit image with an alpha channel
        # ARGB, 32-bit image with alpha channel first
        # RGBA_PREMULT, 32-bit image with colors scaled by alpha channel
        # ARGB_PREMULT, 32-bit image with colors scaled by alpha channel, alpha channel first

        try:
            if RECORDING:
                VIDEO.append(pygame.image.tostring(SCREEN, 'RGB', False))
        except MemoryError:
            print('\n [-] Insufficient Memory to record string buffer !!!')      
            STOP_GAME = True
            RECORDING = False 
              
        FRAME += 1
        pygame.event.clear()

    # Return the size of an object in bytes
    print('\n [+] VIDEO Object size : ', sys.getsizeof(VIDEO)) 
    print('\n [+] RECORDING AVI file ')
    # *** Record the video
    if RECORDING:

        # Parameters
        # filename	Name of the output video file.
        # fourcc	4-character code of codec used to compress the frames. For example,
        #               VideoWriter::fourcc('P','I','M','1') is a MPEG-1 codec, VideoWriter::fourcc('M','J','P','G')
        #               is a motion-jpeg codec etc. List of codes can be obtained at Video Codecs by FOURCC page.
        #               FFMPEG backend with MP4 container natively uses other values as fourcc code: see ObjectType,
        #               so you may receive a warning message from OpenCV about fourcc code conversion.
        # fps	Framerate of the created video stream.
        # frameSize	Size of the video frames.
        # isColor	If it is not zero, the encoder will expect and encode color frames,
        #               otherwise it will work with grayscale frames (the flag is currently supported on Windows only).
        # Tips:

        # With some backends fourcc=-1 pops up the codec selection dialog from the system.
        # To save image sequence use a proper filename (eg. img_%02d.jpg) and fourcc=0 OR fps=0.
        # Use uncompressed image format (eg. img_%02d.BMP) to save raw frames.
        # Most codecs are lossy. If you want lossless video file you need to use a
        # lossless codecs (eg. FFMPEG FFV1, Huffman HFYU, Lagarith LAGS, etc...)
        # If FFMPEG is enabled, using codec=0; fps=0; you can create an uncompressed (raw) video file.
        video = cv2.VideoWriter('YourVideoName.avi',
                                cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 30,
                                (SCREENRECT.w, SCREENRECT.h), True)
                                
                                

        for event in pygame.event.get():
            pygame.event.clear()

        for image in VIDEO:
            
            image = numpy.frombuffer(image, numpy.uint8).reshape(SCREENRECT.h, SCREENRECT.w, 3)

            # Python: cv.CvtColor(src, dst, code) → None
            # Parameters:	
            # src – input image: 8-bit unsigned, 16-bit unsigned ( CV_16UC... ), or single-precision floating-point.
            # dst – output image of the same size and depth as src.
            # code – color space conversion code (see the description below).
            # dstCn – number of channels in the destination image; if the parameter is 0,
            #the number of the channels is derived automatically from src and code .`
            image = cv2.cvtColor(image, COLOR_RGBA2BGR)
            video.write(image)

        cv2.destroyAllWindows()
        video.release()
        
    pygame.quit()
