import numpy as np 
import matplotlib.pyplot as plt

t, z, vz, Tz, x, Tx = np.loadtxt("output/worst.csv", delimiter=",", skiprows=1, unpack=True)
T = np.sqrt(Tx**2 + Tz**2)

plt.plot(t, Tz, label="vertical thrust (Tz)")
plt.plot(t, Tx, label="horizontal thrust (Tx)")
plt.plot(t, T, "--", label="total thrust")
plt.axhline(20000, color="gray", linewidth=0.8, label="Tmax")
plt.xlabel("time (s)")
plt.ylabel("thrust (N)")
plt.legend()
plt.savefig("output/worst_thrust.png")
plt.show()