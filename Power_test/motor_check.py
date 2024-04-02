import RPi.GPIO as GPIO
import time

# Set up GPIO pin for PWM
motor_pin = 18  # Example GPIO pin
GPIO.setmode(GPIO.BCM)
GPIO.setup(motor_pin, GPIO.OUT)

# Create PWM instance with 100Hz frequency
pwm = GPIO.PWM(motor_pin, 100)  # 100 Hz frequency

# Start PWM with 50% duty cycle (adjust as needed)
pwm.start(50)

try:
    # Keep the PWM signal running for a certain time or until a condition is met
    while True:
        time.sleep(1)
        # Adjust duty cycle, frequency, etc., as needed
finally:
    pwm.stop()  # Stop PWM
    GPIO.cleanup()  # Clean up GPIO to ensure a clean exit
