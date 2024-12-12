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

# Variables to store drawn points
drawn_points = []
line_segments = []
# Function to calculate forward kinematics
def forward_kinematics(theta1, theta2):
    x = L1 * cos(theta1) + L2 * cos(theta1 + theta2)
    y = L1 * sin(theta1) + L2 * sin(theta1 + theta2)
    return x, y

# Function to interpolate the path in joint space
def interpolate_joint_path(start_angles, end_angles, num_points=50):
    theta1_points = np.linspace(start_angles[0], end_angles[0], num_points)
    theta2_points = np.linspace(start_angles[1], end_angles[1], num_points)
    path = []

    for theta1, theta2 in zip(theta1_points, theta2_points):
        x, y = forward_kinematics(theta1, theta2)
        path.append((x, y))
    return path

# Function to handle clicks and add points
def on_click(event):
    if event.inaxes != ax:
        return

    # Get clicked coordinates
    x, y = event.xdata, event.ydata
    drawn_points.append((x, y))

    # Check if the point is reachable
    result = inverse_kinematics(x, y)
    if result is not None:
        theta1, theta2 = result
        print(f"Point: ({x:.2f}, {y:.2f}) -> theta1: {np.degrees(theta1):.2f}, theta2: {np.degrees(theta2):.2f}")

        # Draw the point
        ax.plot(x, y, 'go')  # Draw the point as a green dot

        # Simulate end-effector movement
        if len(drawn_points) > 1:
            prev_x, prev_y = drawn_points[-2]
            prev_result = inverse_kinematics(prev_x, prev_y)

            if prev_result is not None:
                prev_theta1, prev_theta2 = prev_result
                # Interpolate in joint space
                interpolated_path = interpolate_joint_path((prev_theta1, prev_theta2), (theta1, theta2))

                # Extract path coordinates for plotting
                path_x, path_y = zip(*interpolated_path)

                # Simulate motor movement
                for px, py in zip(path_x, path_y):
                    ax.plot(px, py, 'b.', markersize=2)  # Draw the movement as a blue dot
                    fig.canvas.draw_idle()
                    plt.pause(0.01)  # Pause to visualize movement

# Connect the click event to the on_click function
cid = fig.canvas.mpl_connect('button_press_event', on_click)

# Show the plot
plt.show()

# Once you are done drawing, you can also output the drawn coordinates and the line segments
print("\nFinal Coordinates and Connections:")
for i, (x, y) in enumerate(drawn_points):
    print(f"Point {i + 1}: ({x:.2f}, {y:.2f})")
