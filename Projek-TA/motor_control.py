import RPi.GPIO as GPIO
from time import sleep

# Pins for Motor Driver Inputs 
Motor1A = 21 #kuning
Motor1B = 20 #orange
kecepatan1 = 16 #merah

Motor2A = 26 #coklat
Motor2B = 13 #ungu
kecepatan2 = 19 #putih
 
def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)              # GPIO Numbering
    GPIO.setup(Motor1A,GPIO.OUT)  # All pins as Outputs
    GPIO.setup(Motor1B,GPIO.OUT)
    GPIO.setup(kecepatan1,GPIO.OUT)
    p1 = GPIO.PWM(kecepatan1,1000)

    GPIO.setup(Motor2A,GPIO.OUT)  # All pins as Outputs
    GPIO.setup(Motor2B,GPIO.OUT)
    GPIO.setup(kecepatan2,GPIO.OUT)
    p2 = GPIO.PWM(kecepatan2,1000)

 
def loop():
    
    # Going forwards
    GPIO.output(Motor1A,GPIO.HIGH)
    GPIO.output(Motor1B,GPIO.LOW)
    GPIO.output(kecepatan1,GPIO.HIGH)
    print("motor1")

    GPIO.output(Motor2A,GPIO.HIGH)
    GPIO.output(Motor2B,GPIO.LOW)
    GPIO.output(kecepatan2,GPIO.HIGH)
    print("motor2")
    
    print("Going forwards")
 
    sleep(5)
    # Going backwards
    GPIO.output(Motor1A,GPIO.LOW)
    GPIO.output(Motor1B,GPIO.HIGH)
    GPIO.output(kecepatan1,GPIO.HIGH)

    GPIO.output(Motor2A,GPIO.LOW)
    GPIO.output(Motor2B,GPIO.HIGH)
    GPIO.output(kecepatan2,GPIO.HIGH)
    #p.ChangeDutyCycle(100)
    print("Going backwards")
 
    sleep(5)
    # Stop
    GPIO.output(kecepatan1,GPIO.LOW)
    GPIO.output(Motor1B,GPIO.LOW)

    GPIO.output(kecepatan2,GPIO.LOW)
    GPIO.output(Motor2B,GPIO.LOW)
    print("Stop")

def destroy():  
    GPIO.cleanup()

if __name__ == '__main__':     # Program start from here
    setup()
    try:
            loop()
    except KeyboardInterrupt:
        destroy()