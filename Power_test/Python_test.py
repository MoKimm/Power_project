import RPi.GPIO as GPIO
import time
import numpy as np

GPIO.setmode(GPIO.BCM)

LED_pin = 17
GPIO.setup(LED_pin, GPIO.OUT)

pwm = GPIO.PWM(LED_pin, 1000)
pwm.start(0)

try:
    hbp = np.array([0,0.5,1,0.5,0])
    for value in hbp:
        duty = value*100
        pwm.ChangeDutyCycle(duty)
        time.sleep(0.01)

finally:
    pwm.stop()
    GPIO.cleanup()
