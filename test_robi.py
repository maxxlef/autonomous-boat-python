import numpy as np
import time


def cercle(t,lat_boue,long_boue,k=0):
    lx,ly=30,30
    r=10 # en m
    T=450 # en s
    N=10
    m = np.array([[50],[50]])
    p = np.array([[lx],[ly]])
    p_tilde= m +r * np.array([[np.cos(2*np.pi*((t)/T+k/N))],
                             [np.sin(2*np.pi*((t)/T+k/N))]])
    
    v_tilde= r * (2*np.pi/T) * np.array([[-np.sin(2*np.pi*((t)/T+k/N))],
                                         [np.cos(2*np.pi*((t)/T+k/N))]])
    
    w= np.tanh(p_tilde - p) + v_tilde
    print("p_tilde : {} \n p : {} \n v_tilde : {} \n w : {}".format(p_tilde,p,v_tilde,w))
    return w
for i in range (100):
    cercle(i,0,0)
    time.sleep(0.5)    
    print("i : {}".format(i))