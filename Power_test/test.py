import numpy as np
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import time

LED_pin = 17  # replace with your GPIO pin number
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_pin, GPIO.OUT)

np.set_printoptions(precision=4,linewidth=160)

def uSignal(t):    return np.array(0.5*np.sign(t) + 0.5,dtype=int)
def wSignal(t,dt): return uSignal(t)-uSignal(t-dt)
def vSignal(t):    return np.maximum(1-np.abs(t),0)
def dSignal(t): DT = t[1]-t[0]; return uSignal(t) - uSignal(t-DT)
def tSignal(t):
    u1 = uSignal(t-0); u2 = uSignal(t-2); u3 = uSignal(t-5); u4 = uSignal(t-7);
    t1 =        (t-0); t2 =        (t-2); t3 =        (t-5); t4 =        (t-7)
    return u1*t1 - u2*t2 - u3*t3 + u4*t4 
def sincSignal(t): y1 = np.sinc(t); v1 = vSignal(np.pi/10.0*t); return y1*v1              # sinc = sin(x)/x
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

def plot_sig1(t,y1,title1):
    fig, (ax1) = plt.subplots(1, 1, sharey=True,figsize=(20,2),facecolor=('#e0e0eb'))
    nt,tmin, tmax = t.shape[0], np.min(t), np.max(t)
    ax1.set_xlim([tmin, tmax]);
    g_zero = -0.05; 
    ax1.plot(t,y1,lw=2);
    ax1.vlines(0,0,1,'r',linestyles='dotted',lw=3)
    ax1.set_title(title1,fontweight='bold'); ax1.margins(0, 0.1)
    plt.show()

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
tidx,t1 = set_δ(t,10*2*np.pi);
hbp = HeartBeat_pattern(t,tidx);
'''
def output_to_GPIO(t, signal, pin):
    dt = t[1] - t[0]  # time difference between samples
    
    for value in signal:
        GPIO.output(pin, GPIO.HIGH if value > 0.5 else GPIO.LOW)
        time.sleep(dt)
try:
    # Build SCG pattern
    tidx, t1 = set_δ(t, 3*2*np.pi)
    hbp = HeartBeat_pattern(t, tidx)
    scaled_hbp = scale01(hbp)  # Make sure the signal is scaled between 0 and 1

    # Output to GPIO
    output_to_GPIO(t, scaled_hbp, LED_pin)

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
#plot it
'''
plot_sig1(tr(t),ar(hbp),'Heart beat (SCG) pattern')


