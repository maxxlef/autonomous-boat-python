import numpy as np
import time


x=np.array([1.0 / 10, 2.0 / 10, 3.0 / 10])
y=np.array([1.0 / 10, 2.0 / 10, 4.0 / 10])
z=np.array([1.0 / 10, 2.0 / 10, 5.0 / 10])


with open("calib_16.txt", "w") as f:
    np.savetxt(f, x.reshape(-1, 1))
    np.savetxt(f, y.reshape(-1, 1))
    np.savetxt(f,z.reshape(-1, 1))
    np.savetxt(f, z.reshape(-1, 1))