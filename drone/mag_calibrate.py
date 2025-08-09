import numpy as np
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 1. Load the raw data collected from mag_gatherdata.py
magdata = np.load("magdata.npy")

# 2. Calculate hard-iron offsets (the center of the data cloud)
mag_min = np.min(magdata, axis=0)
mag_max = np.max(magdata, axis=0)
mag_offset = (mag_max + mag_min) / 2.0

# 3. Calculate soft-iron scaling (the radius of the data cloud)
mag_scale = (mag_max - mag_min) / 2.0
avg_scale = np.mean(mag_scale)
# The correction scales each axis to have the same average radius
mag_scale_correction = avg_scale / mag_scale

# Apply the calibration to the raw data to visualize it
calibrated_mag = (magdata - mag_offset) * mag_scale_correction

# --- Visualization (Optional but recommended) ---
fig = plt.figure(figsize=(15, 7))

# Plot Raw Data
ax1 = fig.add_subplot(121, projection='3d')
ax1.scatter(magdata[:, 0], magdata[:, 1], magdata[:, 2], c='red', s=5, label='Raw Data')
ax1.set_title('Uncalibrated Magnetometer Data')
ax1.set_xlabel('X-axis')
ax1.set_ylabel('Y-axis')
ax1.set_zlabel('Z-axis')
ax1.legend()
ax1.axis('equal')

# Plot Calibrated Data
ax2 = fig.add_subplot(122, projection='3d')
ax2.scatter(calibrated_mag[:, 0], calibrated_mag[:, 1], calibrated_mag[:, 2], c='green', s=5, label='Calibrated Data')
ax2.set_title('Calibrated Magnetometer Data')
ax2.set_xlabel('X-axis')
ax2.set_ylabel('Y-axis')
ax2.set_zlabel('Z-axis')
ax2.legend()
ax2.axis('equal')

plt.show()

# --- Save the CORRECT calibration parameters ---
np.save("magOffset.npy", mag_offset)
# Note: The main script will MULTIPLY by this, not divide.
# We'll adjust the main script slightly.
np.save("magScale.npy", mag_scale_correction)