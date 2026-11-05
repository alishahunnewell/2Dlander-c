import numpy as np
import matplotlib.pyplot as plt

t, z, vz, z_exact = np.loadtxt("output/freefall.csv", delimiter=",", skiprows=1, unpack=True)

plt.plot(t, z, label="RK4")
plt.plot(t, z_exact, "--", label="exact")
plt.xlabel("time (s)")
plt.ylabel("height (m)")
plt.legend()
plt.savefig("output/freefall.png")
plt.show()