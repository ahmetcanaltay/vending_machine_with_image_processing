import time
import tensorflow.keras
from PIL import Image, ImageOps
import numpy as np
import cv2

import RPi.GPIO as GPIO
from RPLCD import CharLCD
import socket
import fcntl
import struct
import digitalio
import board
from keypad import keypad
import serial

#defining serial commmunication

ser=serial.Serial(
port="/dev/ttyUSB0",
baudrate=9600,
parity=serial.PARITY_NONE,
bytesize=serial.EIGHTBITS,
timeout=0.5,
)

#lcd setup
lcd = CharLCD(cols=16, rows=2, pin_rs=37, pin_e=35, pins_data=[33, 31, 29, 23])
#keypad setup
L1 = 5
C1 = 12
C2 = 16
C3 = 20
C4 = 21

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(L1, GPIO.OUT)
GPIO.setup(C1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(C2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(C3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(C4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def readLine(line, characters,button_pus):
GPIO.output(line, GPIO.HIGH)
if(GPIO.input(C1) == 1):
button_pus=characters[0]
if(GPIO.input(C2) == 1):
button_pus=characters[1]
if(GPIO.input(C3) == 1):
button_pus=characters[2]
if(GPIO.input(C4) == 1):
button_pus=characters[3]
GPIO.output(line, GPIO.LOW)


button_pusx=0
key=cv2.waitKey(1)
webcam = cv2.VideoCapture(0)
i=1
account_balance=0
digit=None

while(True):
    np.set_printoptions(suppress=True)
    model = tensorflow.keras.models.load_model('keras_model.h5')
    data = np.ndarray(shape=(1, 224, 224, 3), dtype=np.float32)
    image = Image.open('test_photo.jpg')
    #resize the image to a 224x224 with the same strategy as in TM2:
    #resizing the image to be at least 224x224 and then cropping from the center
    size = (224, 224)
    image = ImageOps.fit(image, size, Image.ANTIALIAS)
    #turn the image into a numpy array
    image_array = np.asarray(image)
    # display the resized image
    image.show()
    # Normalize the image
    normalized_image_array = (image_array.astype(np.float32) / 127.0) - 1
    # Load the image into the array
    data[0] = normalized_image_array
    # run the inference
    prediction = model.predict(data)
    #empty area
    if prediction[0][0] == np.max(prediction):
        print("bos alan")
        #ser.write(b'servo0')
        time.sleep(0.1)
    #detect 1 tl
    elif prediction[0][1]== np.max(prediction):
        print("1 tl")
        ser.write(b'1')
        account_balance=account_balance+100
        time.sleep(0.1)

    #detect 50 kr
    elif prediction[0][2]== np.max(prediction):
        print("50kr")
        ser.write(b'1')
        account_balance=account_balance+50
        time.sleep(0.1)
    #detect 25 kr
    elif prediction[0][3]== np.max(prediction):
        print("25kr")
        ser.write(b'1')
        account_balance=account_balance+25
        time.sleep(0.1)
    #detect 1 tl arka
    elif prediction[0][4]== np.max(prediction):
        print("1 tl")
        ser.write(b'1')
        account_balance=account_balance+100
        time.sleep(0.1)
    #detect 50 kr arka
    elif prediction[0][5]== np.max(prediction):
        print("50kr")
        ser.write(b'1')
        account_balance=account_balance+50
        time.sleep(0.1)
    #detect 25 kr arka
    elif prediction[0][6]== np.max(prediction):
        print("25kr")
        ser.write(b'1')
        account_balance=account_balance+25
        time.sleep(0.1)

    #detect non tl
    else:
        print("tanimsiz para")
        ser.write(b'0')
        time.sleep(0.1)
        print(prediction)
        time.sleep(1)
        i=i+1
    #self break at 10 repeat for test
    if i>50:
        break
    #break loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    #capture image and save
    check, frame = webcam.read()
    cv2.imwrite(filename='test_photo.jpg', img=frame)
    readLine(L1, ["1","2","3","4"],button_pusx)
    #give product
    if(button_pusx==1):
        if(account_balance>99):
            account_balance=account_balance-100
            ser.write(b'2')
        else:
            lcd.write_string('yetersiz bakiye')
            button_pusx=0
    #balance on lcd
    lcd.write_string(account_balance)
    webcam.release()

