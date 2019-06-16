# enable pi camera in the config
# install picamera: --> pip install "picamera[array]"
# or install imutils: --> pip3 install imutils
# install opencv: -->  pip3 install opencv-python
# install missing dependencies (if needed): --> sudo apt-get install -y libcblas-dev libhdf5-dev libhdf5-serial-dev libatlas-base-dev libjasper-dev  libqtgui4  libqt4-test
# install and compile dlib: --> https://www.pyimagesearch.com/2017/05/01/install-dlib-raspberry-pi/
# if needed get haarcascade xml files: can be found in /resources download was currupt!? copied from usb
# install pigpio: --> pip3 install pigpio


#---------- configuration-----------------------------------#

# debugging
measure_fps = True

# select one of these software libraries for camera capture
use_picamera = True
use_imutils = False

# select one of these face detection methods
use_haarcacades = False
use_dlib = True

# enable servos
enable_servos = True
# select one of these servo libraries
use_rpi_gpio = False
use_pigpio = True

#-----------------------------------------------------------#

import time
import cv2
#import tkinter

if use_picamera:
    from picamera.array import PiRGBArray
    from picamera import PiCamera
if use_imutils:
    from imutils.video import VideoStream

if enable_servos:
    if use_rpi_gpio:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.OUT)
        pan_pwm = GPIO.PWM(18, 100)
        pan_pwm.start(50) 
        GPIO.setup(23, GPIO.OUT)
        roll_pwm = GPIO.PWM(23, 100)
        roll_pwm.start(13)
        GPIO.setup(24, GPIO.OUT)
        tilt_pwm = GPIO.PWM(24, 100)
        tilt_pwm.start(13)

    if use_pigpio:
        import pigpio
        pi=pigpio.pi()
        pi.set_mode(18,pigpio.OUTPUT)
        pi.set_servo_pulsewidth(18,1500)

        pi.set_mode(23,pigpio.OUTPUT)
        pi.set_servo_pulsewidth(24,1500)
        
        pi.set_mode(24,pigpio.OUTPUT)
        pi.set_servo_pulsewidth(23,1500)
        

#    if use_wiringpi:
#        import wiringpi
#        wiringpi.wiringPiSetupGpio()
#        wiringpi.pinMode(18, wiringpi.GPIO.PWM_OUTPUT)
#        wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
#        wiringpi.pwmSetClock(192)
#        wiringpi.pwmSetRange(2000)
#        wiringpi.pwmWrite(18, 125)



ppwm = 1500

#from RPIO import PWM
#if enable_servos:
#
#    servo = PWM.Servo()
#    servo.set_servo(18, 1200)



if use_dlib:
    import dlib

width = 640
height = 480

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
    face_cascade =  cv2.CascadeClassifier('/home/pi/Mira/resources/haarcascade_frontalface_alt.xml')
    #face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

if use_dlib:
    detector = dlib.get_frontal_face_detector()
    
if measure_fps:
    start_time = time.time()

if use_picamera:
    for frame in camera.capture_continuous(rawCapture, format="bgr",  use_video_port=True):

    #    image=np.asarray(frame.array)
        img = rawCapture.array
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        xpos = 0

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

        dx = xpos - width/2
        print (xpos, dx, ppwm)
        if dx<-100:
            if ppwm < 2500:
                ppwm += 100
            
        if dx>100:
            if ppwm > 500:
                ppwm -= 100
            
        if enable_servos:    
#            pan_pwm.ChangeDutyCycle(ppwm)
            pi.set_servo_pulsewidth(18,ppwm)
            pi.set_servo_pulsewidth(24,1500)
            pi.set_servo_pulsewidth(23,1500)
        
        cv2.imshow("Image", img)
        rawCapture.truncate(0)
            

        
            
        if measure_fps:
            current_time = time.time()
            print("FPS: ", 1.0/(current_time - start_time))
            start_time = current_time
        
        k = cv2.waitKey(1) & 0xff # escape key
        if k ==27:
            break
    

if use_imutils:
    while True:

    #    image=np.asarray(frame.array)
        img = vs.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        xpos = 0

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

        dx = xpos - width/2
        print (dx, ppwm)
        if dx<-100:
            if ppwm < 30:
                ppwm += 1
            
        if dx>100:
            if ppwm > 10:
                ppwm -= 1
            
        if enable_servos:    
            pan_pwm.ChangeDutyCycle(ppwm)
        

        
        cv2.imshow("Image", img)
            

        
            
        if measure_fps:
            current_time = time.time()
            print("FPS: ", 1.0/(current_time - start_time))
            start_time = current_time
        
        k = cv2.waitKey(1) & 0xff # escape key
        if k ==27:
            break



cv2.destroyAllWindows()

