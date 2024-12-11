import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
from rdp import rdp  # Install with `pip install rdp`
from math import atan2, acos, sqrt, pi

# 1. Load and preprocess the image
image_path = "./1532.jpg"
img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)  # Load as grayscale
img = cv2.resize(img, (28, 28))  # Resize to 28x28
_, img_binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)  # Binarize

# 2. Extract coordinates of white pixels
coords = np.column_stack(np.where(img_binary > 0))

# 3. Map to Cartesian coordinates and scale to robot workspace
coords_cartesian = [(x, 28 - y) for y, x in coords]
scale_factor = 1 / 28.0
coords_scaled = [(x * scale_factor, y * scale_factor) for x, y in coords_cartesian]

# 4. Smooth the path using spline interpolation
x, y = zip(*coords_scaled)
tck, u = splprep([x, y], s=0.5)  # Adjust `s` for smoothing
x_smooth, y_smooth = splev(np.linspace(0, 1, 100), tck)  # 100 points
smoothed_coords = list(zip(x_smooth, y_smooth))

# 5. Simplify the smoothed path
simplified_coords = rdp(smoothed_coords, epsilon=0.01)  # Adjust epsilon for simplification

# 6. Inverse Kinematics (2-DOF example)
def inverse_kinematics_2dof(x, y, l1=10.5, l2=8.75):
    r = sqrt(x**2 + y**2)
    if r > l1 + l2:
        raise ValueError(f"Point ({x}, {y}) is out of reach for the robot.")
    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = acos(np.clip(cos_theta2, -1.0, 1.0))
    theta1 = atan2(y, x) - atan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))
    return theta1, theta2

# 7. Apply IK and validate with FK
def forward_kinematics_2dof(theta1, theta2, l1=10.50, l2=8.750):
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
    return x, y

joint_angles = []
fk_coords = []
for x, y in simplified_coords:
    try:
        angles = inverse_kinematics_2dof(x, y)
        joint_angles.append(angles)
        fk_coords.append(forward_kinematics_2dof(*angles))
    except ValueError as e:
        print(e)

# 8. Plotting
plt.figure(figsize=(8, 6))
plt.plot(x_smooth, y_smooth, '-', label='Smoothed Path', alpha=0.6)
plt.plot(*zip(*simplified_coords), 'o-', label='Simplified Path', alpha=0.8)
plt.plot(*zip(*fk_coords), 'x-', label='Reconstructed Path via FK', alpha=0.8)
plt.xlabel("X (workspace)")
plt.ylabel("Y (workspace)")
plt.title("Path Visualization and IK Validation")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
