import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, acos, atan2, sqrt, pi

# Define link lengths and joint limits
L1 = 8.75  # Length of the first link
L2 = 10.5  # Length of the second link

# Joint angle limits in degrees
JOINT1_LIMITS = (-90, 90)  # Limits for joint 1
JOINT2_LIMITS = (-90, 90)  # Limits for joint 2

# Convert degrees to radians
JOINT1_LIMITS_RAD = np.radians(JOINT1_LIMITS)
JOINT2_LIMITS_RAD = np.radians(JOINT2_LIMITS)

# Function to calculate forward kinematics
def forward_kinematics(theta1, theta2):
    x = L1 * cos(theta1) + L2 * cos(theta1 + theta2)
    y = L1 * sin(theta1) + L2 * sin(theta1 + theta2)
    return x, y

# Function to calculate inverse kinematics
def inverse_kinematics(x, y):
    r = sqrt(x**2 + y**2)
    if r > L1 + L2 or r < abs(L1 - L2):
        return None  # Point is out of reach

    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = acos(np.clip(cos_theta2, -1.0, 1.0))
    theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2))

    return theta1, theta2

# Generate the shape (e.g., square in the shape of "0")
def generate_shape():
    shape = [
        (5, 5), (5, -5), (-5, -5), (-5, 5), (5, 5)  # Coordinates for a square
    ]
    return shape

shape_coords = generate_shape()

# Compute the joint angles for the shape and validate reachability
joint_angles = []
drawable_coords = []
# Check if the inverse kinematics results in valid angles
for x, y in shape_coords:
    angles = inverse_kinematics(x, y)
    if angles is not None:
        theta1, theta2 = angles
        if (JOINT1_LIMITS_RAD[0] <= theta1 <= JOINT1_LIMITS_RAD[1] and
            JOINT2_LIMITS_RAD[0] <= theta2 <= JOINT2_LIMITS_RAD[1]):
            joint_angles.append(angles)
            drawable_coords.append((x, y))
        else:
            print(f"Joint limits exceeded for point ({x}, {y}): theta1={theta1}, theta2={theta2}")
    else:
        print(f"Point ({x}, {y}) is unreachable.")


# Reconstruct the shape using forward kinematics
fk_coords = [forward_kinematics(theta1, theta2) for theta1, theta2 in joint_angles]

# Plot the results
fig, ax = plt.subplots()
plt.title("Drawing Shape with Inverse Kinematics")
plt.xlabel("X (cm)")
plt.ylabel("Y (cm)")
plt.axis('equal')
plt.grid(True)

# Plot the desired shape
shape_x, shape_y = zip(*shape_coords)
ax.plot(shape_x, shape_y, 'r--', label="Desired Shape")

# Plot the reconstructed shape
fk_x, fk_y = zip(*fk_coords)
ax.plot(fk_x, fk_y, 'b-', label="Reconstructed Shape")

# Plot the robot's arm positions for each point
for (theta1, theta2) in joint_angles:
    x1, y1 = L1 * cos(theta1), L1 * sin(theta1)
    x2, y2 = forward_kinematics(theta1, theta2)
    ax.plot([0, x1], [0, y1], 'g-', lw=2)
    ax.plot([x1, x2], [y1, y2], 'm-', lw=2)

# Add legend
plt.legend()
plt.show()
