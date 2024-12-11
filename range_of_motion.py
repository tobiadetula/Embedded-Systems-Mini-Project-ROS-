import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from math import cos, sin, acos, atan2, sqrt, pi

# Define link lengths and joint limits
L1 = 10.5  # Length of the first link
L2 = 8.75  # Length of the second link

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

# Generate workspace
resolution = 100  # Grid resolution
x_range = np.linspace(-(L1 + L2), L1 + L2, resolution)
y_range = np.linspace(-(L1 + L2), L1 + L2, resolution)
reachable_points = []

for x in x_range:
    for y in y_range:
        result = inverse_kinematics(x, y)
        if result is not None:
            theta1, theta2 = result
            if (JOINT1_LIMITS_RAD[0] <= theta1 <= JOINT1_LIMITS_RAD[1] and
                JOINT2_LIMITS_RAD[0] <= theta2 <= JOINT2_LIMITS_RAD[1]):
                reachable_points.append((x, y))

reachable_points = np.array(reachable_points)

# Plot interactive range of motion
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.1, bottom=0.25)

# Plot reachable workspace
ax.plot(reachable_points[:, 0], reachable_points[:, 1], '.', markersize=1, label='Reachable Points')
ax.set_title("Interactive Range of Motion")
ax.set_xlabel("X (cm)")
ax.set_ylabel("Y (cm)")
ax.axis('equal')
ax.grid(True)
ax.legend()

# Add sliders for real-time joint angle visualization
ax_joint1 = plt.axes([0.1, 0.1, 0.65, 0.03], facecolor="lightgoldenrodyellow")
ax_joint2 = plt.axes([0.1, 0.05, 0.65, 0.03], facecolor="lightgoldenrodyellow")

slider_joint1 = Slider(ax_joint1, 'Joint 1', *JOINT1_LIMITS, valinit=0)
slider_joint2 = Slider(ax_joint2, 'Joint 2', *JOINT2_LIMITS, valinit=0)

# Plot robot arm and interactive update
def update(val):
    theta1 = np.radians(slider_joint1.val)
    theta2 = np.radians(slider_joint2.val)

    # Calculate end-effector position
    x1, y1 = L1 * cos(theta1), L1 * sin(theta1)
    x2, y2 = forward_kinematics(theta1, theta2)

    # Clear previous arm plot
    ax.cla()  # Clear axes

    # Replot reachable workspace
    ax.plot(reachable_points[:, 0], reachable_points[:, 1], '.', markersize=1, label='Reachable Points')

    # Draw arm
    ax.plot([0, x1], [0, y1], 'r-', lw=2, label="Link 1")
    ax.plot([x1, x2], [y1, y2], 'b-', lw=2, label="Link 2")
    ax.plot(x2, y2, 'go', label="End-Effector")

    # Redraw plot with updated labels and grid
    ax.set_title("Interactive Range of Motion")
    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax.axis('equal')
    ax.grid(True)
    ax.legend()

    # Refresh plot
    fig.canvas.draw_idle()

# Initialize with default arm position
update(0)

# Connect sliders to update function
slider_joint1.on_changed(update)
slider_joint2.on_changed(update)

plt.show()
