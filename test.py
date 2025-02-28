import cv2

thresholded_image = cv2.imread('img_0.jpg', cv2.IMREAD_GRAYSCALE)  # Ensure it's loaded as grayscale
edged = cv2.Canny(thresholded_image, 50, 100)

# Find contours in the thresholded image
contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

# Iterate through the contours and find bounding boxes
bounding_boxes = []
for contour in contours:
    # Get bounding box coordinates
    x, y, w, h = cv2.boundingRect(contour)
    if w*h > 2500:
        bounding_boxes.append((x, y, w, h))

# Draw bounding boxes on a copy of the original image (for visualization)
original_image = cv2.imread('img_0.jpg')  # Load your original image here
bounding_box_image = original_image.copy()
for (x, y, w, h) in bounding_boxes:
    cv2.rectangle(bounding_box_image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw green rectangles

# Display or save the image with bounding boxes
cv2.imshow('Bounding Boxes', bounding_box_image)

cv2.waitKey(0)
cv2.destroyAllWindows()
# Optionally, you can print the bounding boxes
print("Bounding boxes:", bounding_boxes)
