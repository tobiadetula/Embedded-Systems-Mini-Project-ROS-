import numpy as np
import cv2
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

# Function to calculate inverse kinematics
def inverse_kinematics(x, y):
    r = sqrt(x**2 + y**2)
    if r > L1 + L2 or r < abs(L1 - L2):
        return None  # Point is out of reach

    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = acos(np.clip(cos_theta2, -1.0, 1.0))
    theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2))

    return theta1, theta2

# Function to process image and extract coordinates
def get_image_coords(image_path):
    # Load image and convert to grayscale
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    # Threshold to create a binary image (you can adjust the threshold value)
    _, binary_image = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

    # Find contours (edges of the shape in the image)
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Extract the coordinates of the contour
    coords = []
    for contour in contours:
        for point in contour:
            x, y = point[0]  # Extract x, y coordinates from the contour point
            coords.append((x, y))
    
    return coords

# Function to generate valid coordinates from image based on motor constraints
def generate_valid_coords_from_image(image_path):
    shape_coords = get_image_coords(image_path)  # Extract coordinates from image
    valid_coords = []

    # Check each coordinate to see if it is reachable
    for x, y in shape_coords:
        # Adjust x and y to match your coordinate frame if needed (e.g., scale or translate)
        angles = inverse_kinematics(x, y)
        if angles is not None:
            theta1, theta2 = angles
            # Ensure the joint angles are within the limits
            if (JOINT1_LIMITS_RAD[0] <= theta1 <= JOINT1_LIMITS_RAD[1] and
                JOINT2_LIMITS_RAD[0] <= theta2 <= JOINT2_LIMITS_RAD[1]):
                valid_coords.append((x, y))
            else:
                print(f"Joint limits exceeded for point ({x}, {y}): theta1={theta1}, theta2={theta2}")
        else:
            print(f"Point ({x}, {y}) is unreachable.")
    
    return valid_coords

# Function to plot the valid coordinates and original image shape
def plot_coords(image_path, valid_coords):
    shape_coords = get_image_coords(image_path)  # Extract coordinates from image
    
    # Plotting
    plt.figure(figsize=(6, 6))
    plt.title("Shape Coordinates within Motor Constraints")
    plt.xlabel("X (cm)")
    plt.ylabel("Y (cm)")
    plt.axis('equal')
    plt.grid(True)

    # Plot the original shape (contours from the image)
    shape_x, shape_y = zip(*shape_coords) if shape_coords else ([], [])
    # plt.scatter(shape_x, shape_y, color='red', label="Original Shape", alpha=0.6)

    # Plot the valid coordinates (within motor constraints)
    valid_x, valid_y = zip(*valid_coords) if valid_coords else ([], [])
    plt.scatter(valid_x, valid_y, color='green', label="Valid Coordinates", alpha=0.8)

    # Add legend
    plt.legend()
    plt.show()

# Example usage
if __name__ == "__main__":
    image_path = 'images.png'  # Specify the image path
    valid_coords = generate_valid_coords_from_image(image_path)
    print("Valid coordinates within motor constraints:", valid_coords)
    
    # Plot the coordinates
    plot_coords(image_path, valid_coords)
