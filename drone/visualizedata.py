import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

mag_data = np.load("magdata.npy")

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(mag_data[:, 0], mag_data[:, 1], mag_data[:, 2], c='blue', s=10)
ax.set_title("Raw Magnetometer Readings")
ax.set_xlabel("Mag X")
ax.set_ylabel("Mag Y")
ax.set_zlabel("Mag Z")
plt.show()