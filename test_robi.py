import time 

t0=time.time()
i=0
def mission_jour_2(t0):
    iteration(t0,i)


def iteration(t0,i):
    while time.time()-t0 < 1:
        try:
            i+=1
            print(i)
            time.sleep(0.2)
        except:
            pass
def full_mission(t0):
    mission_jour_2(t0)
    t0=time.time()
    mission_jour_2(t0)
    
full_mission(t0)