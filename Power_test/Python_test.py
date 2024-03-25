import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

LED_pin = 17
GPIO.setup(LED_pin, GPIO.OUT)

GPIO.output(LED_pin, GPIO.HIGH)
print("LED is ON")

time.sleep(50)

GPIO.output(LED_pin, GPIO.LOW)
print("LED is OFF")
