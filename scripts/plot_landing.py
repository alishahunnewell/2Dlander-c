import numpy as np 
import matplotlib.pyplot as plt

runs = [
    ("output/landing_base.csv", "Kp=1, Kd=2"),
    ("output/landing_kd0.csv",  "Kp=1, Kd=0"),
    ("output/landing_kp2.csv",  "Kp=2, Kd=2"),
]

for path, label in runs:
    t, z, vz, Tz = np.loadtxt(path, delimiter=",", skiprows=1, unpack=True)
    plt.plot(t, z, label=label)

plt.axhline(0, color="gray", linewidth=0.5)
plt.xlabel("time (s)")
plt.ylabel("height (m)")
plt.legend()
plt.savefig("output/landing_z.png")
plt.show()