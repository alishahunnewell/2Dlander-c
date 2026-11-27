import numpy as np
import matplotlib.pyplot as plt

runs = [
    ("output/landing_run0.csv", "baseline (landed)"),
    ("output/landing_run1.csv", "no horizontal damper"),
    ("output/landing_run2.csv", "no horizontal control"),
]

for path, label in runs:
    t, z, vz, Tz, x, Tx = np.loadtxt(path, delimiter=",", skiprows=1, unpack=True)
    plt.plot(x, z, label=label)

plt.axhline(0, color="gray", linewidth=0.5)
plt.plot([-5, 5], [0, 0], color="black", linewidth=4, label="pad")
plt.xlabel("x (m)")
plt.ylabel("height (m)")
plt.legend()
plt.savefig("output/landing_path.png")
plt.show()