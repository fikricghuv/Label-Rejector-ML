#Import modul time
import time  
#Import modul RPi.GPIO
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
#Set penomeran pin dengan standar Broadcom  
GPIO.setmode(GPIO.BCM)
#Setup Pin PWM
GPIO.setup(16, GPIO.OUT)  
#Setup Pin kontrol maju mundur
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

#Maju
GPIO.output(20, GPIO.HIGH)
GPIO.output(21, GPIO.LOW) 

#Mundur
#GPIO.output(22, GPIO.LOW)
#GPIO.output(27, GPIO.HIGH)  

#Buat instansiasi pin PWM, jika menggunakan dua motor
#namannya bisa diubah menjadi p1 atau p2, bisa juga motor1 dan motor2
p = GPIO.PWM(16, 100)

#Lakukan perulangan dengan menaikan dan menurunkan nilai PWM motor
#sehingga motor bergerak bertambah cepat lalu melambat dan seterusnya
try:
    p.start(100)         
    while 1:
        for dc in range(0, 101, 5):
            #Perintah untuk mengubah kecepatan motor, dc bisa bernilai 0 - 100
            p.ChangeDutyCycle(dc)
            time.sleep(0.1)
        for dc in range(100, -1, -5):
            p.ChangeDutyCycle(dc)
            time.sleep(0.1)

except KeyboardInterrupt:
    #Stop penggunaan pwm
    p.stop()
    #Reset gpio dan keluar dari program
    GPIO.cleanup() 