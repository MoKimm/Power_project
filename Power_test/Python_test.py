import RPi.GPIO as GPIO
import time
import numpy as np

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
LED_pin = 17
GPIO.setup(LED_pin, GPIO.OUT)
print("on")
GPIO.output(LED_pin, GPIO.HIGH)
time.sleep(1)
print("off")
GPIO.output(LED_pin, GPIO.LOW)

