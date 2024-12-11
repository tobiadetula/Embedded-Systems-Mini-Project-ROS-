import cv2
import numpy as np

# Load and preprocess the image
image_path = "1532.jpg"
img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)  # Load as grayscale
img = cv2.resize(img, (28, 28))  # Resize to 28x28
_, img_binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)  # Binarize

# Extract coordinates of white pixels (non-zero areas)
coords = np.column_stack(np.where(img_binary > 0))

# Map to Cartesian coordinates (optional: invert y-axis)
coords_cartesian = [(x, 28 - y) for y, x in coords]

# Scale to robot workspace (e.g., scale 0-28 to 0-1)
scale_factor = 1 / 28.0
coords_scaled = [(x * scale_factor, y * scale_factor) for x, y in coords_cartesian]

# Smooth the path (optional: use spline interpolation)
from scipy.interpolate import splprep, splev

x, y = zip(*coords_scaled)
tck, u = splprep([x, y], s=0.5)  # Adjust `s` for smoothing
x_smooth, y_smooth = splev(np.linspace(0, 1, 100), tck)  # 100 points

# Save coordinates or send to robot IK
coordinates = list(zip(x_smooth, y_smooth))
for point in coordinates:
    print(point)  # Replace with your IK solver call
