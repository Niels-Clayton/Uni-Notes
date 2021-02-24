import numpy as np
import matplotlib.pyplot as plt
import scipy 

t = np.linspace(0,np.pi * 4, 10000)
bean = np.random.randn(10000)
legume = np.sin(t) * (np.cos(t)+1)

plt.plot(legume)
plt.show()