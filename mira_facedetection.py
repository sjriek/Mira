# enable pi camera in the config
# install picamera: --> pip install "picamera[array]"
# install opencv: -->  pip3 install opencv-python
# install missing dependencies (if needed): --> sudo apt-get install -y libcblas-dev libhdf5-dev libhdf5-serial-dev libatlas-base-dev libjasper-dev  libqtgui4  libqt4-test
# install dlib: --> https://www.pyimagesearch.com/2017/05/01/install-dlib-raspberry-pi/


use_haarcacades = False
use_dlib = True
measure_fps = True
enable_servos = False

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

#import wiringpi
#
#if enable_servos:
#    wiringpi.wiringPiSetupGpio()
#    wiringpi.pinMode(18, wiringpi.GPIO.PWM_OUTPUT)
#    
#    wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
#    wiringpi.pwmSetClock(192)
#    wiringpi.pwmSetRange(2000)
#    
#    wiringpi.pwmWrite(18, 125)

import RPi.GPIO as GPIO
if enable_servos:
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


ppwm = 20

#from RPIO import PWM
#if enable_servos:
#
#    servo = PWM.Servo()
#    servo.set_servo(18, 1200)



if use_dlib:
    import dlib


camera = PiCamera()

width = 640
height = 480
camera.resolution = (width,height)
#camera.resolution = (1280,720)
#camera.framerate = 32
# camera.annotate_text = "Mira"

rawCapture = PiRGBArray(camera)


time.sleep(0.1)

if use_haarcacades:
    face_cascade =  cv2.CascadeClassifier('/home/pi/Mira/resources/haarcascade_frontalface_alt.xml')
    #face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

if use_dlib:
    detector = dlib.get_frontal_face_detector()
    
if measure_fps:
    start_time = time.time()


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
    rawCapture.truncate(0)
        

    
        
    if measure_fps:
        current_time = time.time()
        print("FPS: ", 1.0/(current_time - start_time))
        start_time = current_time
    
    k = cv2.waitKey(1) & 0xff # escape key
    if k ==27:
        break
    

cv2.destroyAllWindows()

