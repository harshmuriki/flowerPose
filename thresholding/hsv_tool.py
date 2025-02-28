import cv2
import numpy as np
# Mouse callback function

all_hsv_values = []

def get_hsv_values(x, y):
    pixel_hsv = hsv_image[x, y]
    print("x: {0}, y: {1} is:".format(x, y))
    print(pixel_hsv)
    all_hsv_values.append(pixel_hsv)
    print("-"*20)

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Clicked at (x={x}, y={y})")
        get_hsv_values(x, y)

def get_values(all_hsv_values):
    print(all_hsv_values)
    all_hsv_values = np.array(all_hsv_values)
    mean = np.mean(all_hsv_values, axis=0)
    max_vals = np.max(all_hsv_values, axis=0)
    # max_1_H = np.max(all_hsv_values[np.where(all_hsv_values[:, 0] < 90)][:, 0])
    # min_2_H = np.min(all_hsv_values[np.where(all_hsv_values[:, 0] >= 90)][:, 0])
    indexes = np.where(all_hsv_values[:, 0] < 90)[0]
    if indexes.size > 0:
    # Compute the minimum value of the first column where the first element is greater than or equal to 90
        max_1_H = np.min(all_hsv_values[np.where(all_hsv_values[:, 0] < 90)][:, 0])
    else:
        max_1_H = None

    # Check if there are any elements satisfying the condition
    if all_hsv_values[np.where(all_hsv_values[:, 0] >= 90)].size > 0:
        # Compute the minimum value of the first column where the first element is less than 90
        min_2_H = np.min(all_hsv_values[np.where(all_hsv_values[:, 0] >= 90)][:, 0])
    else:
        min_2_H = None

    min_vals = np.min(all_hsv_values, axis=0)

    H_vals = (min_vals[0], max_1_H), (min_2_H, max_vals[0])
    S_vals = (min_vals[1], max_vals[1])
    V_vals = (min_vals[2], max_vals[2])

    print("mean", mean)
    print("max", max_vals)
    print("min", min_vals)
    print("H", H_vals)
    print("S:", S_vals)
    print("V:", V_vals)

# Load the image
image = cv2.imread('pic1.png')

hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Create a window and set the mouse callback function
cv2.namedWindow('image')
cv2.setMouseCallback('image', mouse_callback)

while True:
    # Display the image

    cv2.imshow('image', image)
    
    # Wait for a key press
    key = cv2.waitKey(1) & 0xFF
    
    # Break the loop if 'q' is pressed
    if key == ord('q'):
        get_values(all_hsv_values)
        break

# Close all OpenCV windows
cv2.destroyAllWindows()
