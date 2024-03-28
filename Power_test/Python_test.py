'''import RPi.GPIO as GPIO
import time
import numpy as np

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
LED_pin = 17
GPIO.setup(LED_pin, GPIO.OUT)
print("on")
GPIO.output(LED_pin, GPIO.HIGH)
time.sleep(5)
print("off")
GPIO.output(LED_pin, GPIO.LOW)
time.sleep(5)
'''
import simpleaudio as sa

def play_wav_file(wav_path):
    wave_obj = sa.WaveObject.from_wave_file(wav_path)
    play_obj = wave_obj.play()
    play_obj.wait_done()  # Wait until the file has finished playing

# Play the heartbeat WAV file
wav_file_path = 'heartbeat_signal.wav'  # Specify the same path used for saving
play_wav_file(wav_file_path)
