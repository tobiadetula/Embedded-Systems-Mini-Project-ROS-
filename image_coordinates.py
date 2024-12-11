import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
from rdp import rdp  # Install with `pip install rdp`
from math import atan2, acos, sqrt, pi, cos, sin, degrees, radians

# 1. Load and preprocess the image
image_path = "MNIST_JPG_training/2/5.jpg"
img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)  # Load as grayscale
img = cv2.resize(img, (28, 28))  # Resize to 28x28
_, img_binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)  # Binarize

# 2. Extract coordinates of white pixels
coords = np.column_stack(np.where(img_binary > 0))

# 3. Map to Cartesian coordinates and scale to robot workspace
coords_cartesian = [(x, 28 - y) for y, x in coords]
l1, l2 = 10.5, 8.75  # Link lengths
max_reach = l1 + l2
scale_factor = max_reach / 28.0
coords_scaled = [(x * scale_factor, y * scale_factor) for x, y in coords_cartesian]

# 4. Sort points in rotational order
centroid = np.mean(coords_scaled, axis=0)  # Calculate the centroid
def polar_angle(point):
    dx, dy = point[0] - centroid[0], point[1] - centroid[1]
    return atan2(dy, dx)

coords_sorted = sorted(coords_scaled, key=polar_angle)

# 5. Smooth the path using spline interpolation
x, y = zip(*coords_sorted)
tck, u = splprep([x, y], s=1.0)  # Increase `s` for smoother paths
x_smooth, y_smooth = splev(np.linspace(0, 1, 100), tck)
smoothed_coords = list(zip(x_smooth, y_smooth))

# 6. Simplify the smoothed path
simplified_coords = rdp(smoothed_coords, epsilon=0.05)  # Increase epsilon for more aggressive simplification

# 7. Constrain points to robot's reachable workspace and joint limits
def constrain_to_workspace(x, y, l1=10.5, l2=8.75):
    r = sqrt(x**2 + y**2)
    if r > l1 + l2:  # Outside maximum reach
        scale = (l1 + l2) / r
        x, y = x * scale, y * scale
    elif r < abs(l1 - l2):  # Inside minimum reach
        scale = abs(l1 - l2) / r
        x, y = x * scale, y * scale
    return x, y

def adjust_to_joint_limits(x, y, l1=10.5, l2=8.75):
    # Apply IK and validate joint limits
    try:
        theta1, theta2 = inverse_kinematics_2dof(x, y, l1, l2)
        return forward_kinematics_2dof(theta1, theta2, l1, l2)
    except ValueError:
        # Find closest valid point within reach and joint limits
        x, y = constrain_to_workspace(x, y, l1, l2)
        return x, y

# 8. Inverse Kinematics (with joint limit check)
def inverse_kinematics_2dof(x, y, l1=10.5, l2=8.75):
    r = sqrt(x**2 + y**2)
    if r > l1 + l2:
        raise ValueError(f"Point ({x}, {y}) is out of reach for the robot.")
    
    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = acos(np.clip(cos_theta2, -1.0, 1.0))
    theta1 = atan2(y, x) - atan2(l2 * sin(theta2), l1 + l2 * cos(theta2))
    
    # Convert to degrees and check joint limits
    theta1_deg = degrees(theta1)
    theta2_deg = degrees(theta2)
    if not (-45 <= theta1_deg <= 45 and -45 <= theta2_deg <= 45):  # 90° total range
        raise ValueError(f"Point ({x}, {y}) exceeds joint limits: θ1={theta1_deg:.2f}, θ2={theta2_deg:.2f}.")
    
    return theta1, theta2

# 9. Forward Kinematics
def forward_kinematics_2dof(theta1, theta2, l1=10.50, l2=8.750):
    x = l1 * cos(theta1) + l2 * cos(theta1 + theta2)
    y = l1 * sin(theta1) + l2 * sin(theta1 + theta2)
    return x, y

# 10. Adjust points and validate with IK
adjusted_coords = []
for x, y in simplified_coords:
    x_adj, y_adj = adjust_to_joint_limits(x, y)
    adjusted_coords.append((x_adj, y_adj))

# 11. Plot the adjusted path
plt.figure(figsize=(8, 6))
plt.plot(*zip(*coords_sorted), '-', label='Original Path', alpha=0.6)
plt.plot(*zip(*adjusted_coords), 'o-', label='Adjusted Path', alpha=0.8)
plt.xlabel("X (workspace)")
plt.ylabel("Y (workspace)")
plt.title("Path with Adjusted Points within Robot Range")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
