import numpy as np
import RPi.GPIO as GPIO
import time
#import matplotlib.pyplot as plt
np.set_printoptions(precision=4,linewidth=160)

GPIO.setmode(GPIO.BCM)
IN1 = 17  # Control pin for L293D Input 1
IN2 = 27  # Control pin for L293D Input 2
PWM_PIN = 18  # GPIO pin for PWM signal

# Setup GPIO pins for L293D
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(PWM_PIN, GPIO.OUT)  
pwm = GPIO.PWM(PWM_PIN, 100)  # Initialize PWM on PWM_PIN at 100 Hz


pwm.start(0)

def uSignal(t):    return np.array(0.5*np.sign(t) + 0.5,dtype=int)
def wSignal(t,dt): return uSignal(t)-uSignal(t-dt)
def vSignal(t):    return np.maximum(1-np.abs(t),0)
def dSignal(t): DT = t[1]-t[0]; return uSignal(t) - uSignal(t-DT)
def tSignal(t):
    u1 = uSignal(t-0); u2 = uSignal(t-2); u3 = uSignal(t-5); u4 = uSignal(t-7);
    t1 =        (t-0); t2 =        (t-2); t3 =        (t-5); t4 =        (t-7)
    return u1*t1 - u2*t2 - u3*t3 + u4*t4 
def sincSignal(t): y1 = np.sinc(t); v1 = vSignal(np.pi/10*t); return y1*v1              # sinc = sin(x)/x
def sinquadSignal(t): s = np.sin(t-np.pi/2)**2; w = wSignal(t+np.pi/2,np.pi); return s*w  # sin(x)**2

def v_pattern(t,tidx):
    V = np.zeros_like(t)
    for j in tidx: V = V + vSignal(t-t[j])
    return V

def w_pattern(t,tidx,widthW):
    W = np.zeros_like(t)
    for j in tidx: W = W + wSignal(t-t[j],widthW)
    return W

def sinc_pattern(t,tidx): 
    S = np.zeros_like(t) 
    for j in tidx: S = S + sincSignal(t-t[j]); 
    return S

def set_δ(t,ntau):  
    nt,tmin,tmax = t.shape[0], np.min(t), np.max(t);  LT = tmax-tmin; dt = LT/(nt-1)
    δτ = 1.0;    m_step = int(round(ntau * δτ/dt)) #--- guess the integer 'time step' of the array index
    τo = 0;  # set-off                             #--- set the Dirac series
    tidx = np.arange(τo,nt,m_step)
    t1 = np.zeros_like(t).astype(int); t1[τo:nt:m_step] = 1
    return tidx, t1

def scale01(a): return (a-a.min())/(a.max()-a.min())
def scale0(a): return (a-0)/(a.max()-0)

def convolution(t,s1,s2):
    conv = np.zeros_like(s1)
    jt = np.arange(len(s1),dtype=int)
    for i,tt in enumerate(t):
        conv[i]= np.sum(s1[i-jt]*s2[jt])
    return scale01(conv)

nt = 301
it = np.linspace(0,1,nt)
t = 20*(it-0.3)


def HeartBeatSignal(t):
    v1width = 0.2 * np.pi
    v1shift = 0.05 * np.pi
    y1 = np.sinc(2.0 * t)
    v1 = vSignal(v1width * (t - v1shift))
    yv1 = y1 * v1
    S0 = scale0(v1 * y1)
    sf1 = 1.5 * np.pi
    A1 = 0.12
    sq1 = A1 * sinquadSignal(np.exp(0.5 * (t + sf1)) - np.pi / 2)
    sf2 = 1.5 * np.pi
    A2 = 0.20
    sq2 = A2 * sinquadSignal(np.exp(0.5 * (t - sf2)) - np.pi / 2)
    return sq1 + S0 + sq2


#define t
nt, it, thor  = 301, np.linspace(0,1,nt), 5               
t = thor*2*np.pi*(it-0.3)

def HeartBeat_pattern(t,tidx):
    V = np.zeros_like(t)
    for j in tidx: V = V + HeartBeatSignal(t-t[j])
    return V

def ar(u): np.random.seed(1234); return u*(1+0.080*np.random.randn(nt))
def tr(u): np.random.seed(1234); return u*(1+0.001*np.random.randn(nt))

#define t
nt, it, thor  = 301, np.linspace(0,1,nt), 30           
t = thor*2*np.pi*(it-0)

#build SCG pattern
tidx,t1 = set_δ(t,6*2*np.pi);
hbp = HeartBeat_pattern(t,tidx);



def generate_heartbeat_pattern(length):
    t = np.linspace(-3 * np.pi, 3 * np.pi, length)
    return HeartBeatSignal(t)

def motor_control(signal):
    pwm.start(0)
    try:
        for value in signal:
            duty_cycle = max(0, min(100, np.abs(value) * 100 / np.max(np.abs(signal))))  # Normalize to 0-100
            GPIO.output(IN1, GPIO.HIGH if value >= 0 else GPIO.LOW)
            GPIO.output(IN2, GPIO.LOW if value >= 0 else GPIO.HIGH)
            pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.1)  # Adjust as necessary for timing
    finally:
        pwm.stop()
        GPIO.cleanup()

# Main execution block
if __name__ == "__main__":
    signal_length = 100  # Define the length of the signal array
    heartbeat_signal = generate_heartbeat_pattern(signal_length)
    motor_control(heartbeat_signal)
