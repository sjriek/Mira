# enable pi camera in the config
# install picamera: --> pip install "picamera[array]"
# or install imutils: --> pip3 install imutils
# install opencv: -->  pip3 install opencv-python
# install missing dependencies (if needed): --> sudo apt-get install -y libcblas-dev libhdf5-dev libhdf5-serial-dev libatlas-base-dev libjasper-dev  libqtgui4  libqt4-test
# install and compile dlib: --> https://www.pyimagesearch.com/2017/05/01/install-dlib-raspberry-pi/
# You may encounter an "undefined symbol: __atomic_fetch_add8" for libatomic  ---> pip3 install opencv-contrib-python==4.1.0.25
# install cvlib --> pip3 install cvlib 
# if needed get haarcascade xml files: can be found in /resources download was currupt!? copied from usb
# install pigpio: --> pip3 install pigpio



#FPS measurement RPI4
# Picamera, HaarCascades: 7.1 FPS : 320x240 29 fps
# Picamera, dlib : 2,86 FPS : 320x240 7.31 fps (detection distance loss)
# picamera, CVlib: 2,95 FPS : 320x240 3.28 fps

#ImUtils, Haarcascades: 10.75 FPS : 320x240 42 fps
#ImUtils, Dlib: 3.63 FPS : 320x240 14.5 fps (detection distance loss)
#Imutils, CVlib: 2,85 FPS : 320x240 3,68 fps



#---------- configuration-----------------------------------#

# debugging
measure_fps = True

# select one of these software libraries for camera capture
use_picamera = False
use_imutils = True

# select one of these face detection methods
use_haarcacades = False
use_dlib = False
use_cvlib = True

# enable servos
enable_servos = False
# select one of these servo libraries
use_rpi_gpio = True
use_pigpio = False # to get this to work you must rund the gpoiod deamon --> sudo pigpiod


#-----------------------------------------------------------#

import time
import cv2
import os # needed to start the pigpio deamon
#import tkinter

# assign GPIO pinout numbers to constants
SERVO_PAN = 18
SERVO_TILT = 24
SERVO_ROLL = 23

PAN_CENTRE = pan_pwm = 1500
TILT_CENTRE = tilt_pwm = 1500
ROLL_CENTRE = roll_pwm = 1500

if use_picamera:
    from picamera.array import PiRGBArray
    from picamera import PiCamera
if use_imutils:
    from imutils.video import VideoStream

if enable_servos:
    if use_rpi_gpio:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_PAN, GPIO.OUT)
        pan_pwm = GPIO.PWM(SERVO_PAN, 100)
        pan_pwm.start(50) 
        GPIO.setup(SERVO_ROLL, GPIO.OUT)
        roll_pwm = GPIO.PWM(SERVO_ROLL, 100)
        roll_pwm.start(13)
        GPIO.setup(SERVO_TILT, GPIO.OUT)
        tilt_pwm = GPIO.PWM(SERVO_TILT, 100)
        tilt_pwm.start(13)

    if use_pigpio:
        import pigpio
        os.system("sudo pigpiod")
        time.sleep(1)
        pi=pigpio.pi()
        pi.set_mode(SERVO_PAN,pigpio.OUTPUT)
        pi.set_servo_pulsewidth(SERVO_PAN,PAN_CENTRE)
        pi.set_mode(SERVO_TILT,pigpio.OUTPUT)
        pi.set_servo_pulsewidth(SERVO_TILT,TILT_CENTRE)
        pi.set_mode(SERVO_ROLL,pigpio.OUTPUT)
        pi.set_servo_pulsewidth(SERVO_ROLL,ROLL_CENTRE)
        

        

#    if use_wiringpi:
#        import wiringpi
#        wiringpi.wiringPiSetupGpio()
#        wiringpi.pinMode(18, wiringpi.GPIO.PWM_OUTPUT)
#        wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
#        wiringpi.pwmSetClock(192)
#        wiringpi.pwmSetRange(2000)
#        wiringpi.pwmWrite(18, 125)






#from RPIO import PWM
#if enable_servos:
#
#    servo = PWM.Servo()
#    servo.set_servo(18, 1200)





width = 320
height = 240
#width = 640
#height = 480


if use_picamera:
    camera = PiCamera()
    camera.resolution = (width,height)
#camera.resolution = (1280,720)
#camera.framerate = 32
# camera.annotate_text = "Mira"
    rawCapture = PiRGBArray(camera)

if use_imutils:
    vs = VideoStream(usePiCamera=True, resolution=(width, height)).start()

time.sleep(1.0) # wait for camera

if use_haarcacades:
    face_cascade =  cv2.CascadeClassifier('/home/pi/python/Mira/resources/haarcascade_frontalface_alt.xml')
    #face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

if use_dlib:
    import dlib
    detector = dlib.get_frontal_face_detector()
    
if use_cvlib:
    import cvlib as cv
    
if measure_fps:
    start_time = time.time()

if use_picamera:
    ppwm = 0
    if measure_fps:
        HFS = 0
        frames = 0
        
    
    for frame in camera.capture_continuous(rawCapture, format="bgr",  use_video_port=True):

    #    image=np.asarray(frame.array)
        img = rawCapture.array
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        xpos = 0
        ypos = 0

        if use_haarcacades:
    #        print (face_cascade.empty())
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)
            for (x,y,w,h) in faces:
                img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
                roi_gray = gray[y:y+h, x:x+w]
                roi_color = img[y:y+h, x:x+w]
                xpos = (x+w/2)
                
                
                
        if use_dlib:
            faces = detector(gray)
            for face in faces:
                x = face.left()
                y = face.top()
                w = face.right() - x
                h = face.bottom() - y
                img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
                roi_gray = gray[y:y+h, x:x+w]
                roi_color = img[y:y+h, x:x+w]
                xpos = (x+w/2)


        if use_cvlib:
            faces, confidences = cv.detect_face(img)
            for face,conf in zip(faces,confidences):
                x = face[0]
                y = face[1]
                w = face[2] - x
                h = face[3] - y
                #img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
                roi_gray = gray[y:y+h, x:x+w]
                roi_color = img[y:y+h, x:x+w]
                xpos = (x+w/2)

        dx = xpos - width/2
        
        #print (xpos, dx, ppwm)
        if dx<-100:
            if ppwm < 2500:
                ppwm += 100
            
        if dx>100:
            if ppwm > 500:
                ppwm -= 100
            
        if enable_servos:    
#            pan_pwm.ChangeDutyCycle(ppwm)
            pi.set_servo_pulsewidth(SERVO_PAN,PAN_CENTRE)
            pi.set_servo_pulsewidth(SERVO_TILT,TILT_CENTRE)
            pi.set_servo_pulsewidth(SERVO_ROLL,ROLL_CENTRE)
        
        cv2.imshow("Image", img)
        rawCapture.truncate(0)
            

        
            
        if measure_fps:
            current_time = time.time()
            FPS = 1.0/(current_time - start_time)
            #print("FPS: ", 1.0/(current_time - start_time))
            frames = frames+1
            HFS = HFS + FPS
            if frames == 100:
                print ("HFS: ", HFS, " Avg over 100 measurements: ", HFS/100)
                HFS= 0
                frames = 0
              
            start_time = current_time
        
        k = cv2.waitKey(1) & 0xff # escape key
        if k ==27:
            break
    

if use_imutils:
    
    if measure_fps:
        HFS = 0
        frames = 0
        
    while True:

    #    image=np.asarray(frame.array)
        img = vs.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        xpos = 0
        ypos = 0

        if use_haarcacades:
    #        print (face_cascade.empty())
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)
            for (x,y,w,h) in faces:
                img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
                roi_gray = gray[y:y+h, x:x+w]
                roi_color = img[y:y+h, x:x+w]
                xpos = (x+ w/2)
                ypos = (y+ h/2)
                
        if use_dlib:
            faces = detector(gray)
            for face in faces:
                x = face.left()
                y = face.top()
                w = face.right() - x
                h = face.bottom() - y
                img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
                roi_gray = gray[y:y+h, x:x+w]
                roi_color = img[y:y+h, x:x+w]
                xpos = (x+ w/2)
                ypos = (y+ h/2)

        if use_cvlib:
            faces, confidences = cv.detect_face(img)
            for face,conf in zip(faces,confidences):
                x = face[0]
                y = face[1]
                w = face[2] - x
                h = face[3] - y
                img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
                roi_gray = gray[y:y+h, x:x+w]
                roi_color = img[y:y+h, x:x+w]
                xpos = (x+w/2)         


        pmw_stepsize = 25
        detection_treshold = 50
        upper_servo_limit = 2500
        lower_servo_limit = 500
        width_detection_range = width/2
        height_detection_range = height/2

        dx = xpos - width/2
        if -width_detection_range<dx<-detection_treshold:
            if pan_pwm < upper_servo_limit:
                pan_pwm += pmw_stepsize
        if width_detection_range>dx>detection_treshold:
            if pan_pwm > lower_servo_limit:
                pan_pwm -= pmw_stepsize
#        print (xpos, dx, pan_pwm)

        dy = ypos - height/2
        if -height_detection_range<dy<-detection_treshold:
            if tilt_pwm < upper_servo_limit:
                tilt_pwm += pmw_stepsize
        if height_detection_range>dy>detection_treshold:
            if tilt_pwm > lower_servo_limit:
                tilt_pwm -= pmw_stepsize
        #print (ypos, dy, tilt_pwm)



        if enable_servos:    
#            pan_pwm.ChangeDutyCycle(ppwm)
            pi.set_servo_pulsewidth(SERVO_PAN,pan_pwm)
            pi.set_servo_pulsewidth(SERVO_TILT,tilt_pwm)
            pi.set_servo_pulsewidth(SERVO_ROLL,ROLL_CENTRE)      

        
        cv2.imshow("Image", img)
            

        
            
        if measure_fps:
            current_time = time.time()
            FPS = 1.0/(current_time - start_time)
            #print("FPS: ", 1.0/(current_time - start_time))
            frames = frames+1
            HFS = HFS + FPS
            if frames == 100:
                print ("HFS: ", HFS, " Avg over 100 measurements: ", HFS/100)
                HFS= 0
                frames = 0
            start_time = current_time
        
        k = cv2.waitKey(1) & 0xff # escape key
        if k ==27:
            break



cv2.destroyAllWindows()

