import cv2
import numpy as np

# Load the sample image of the poop
img = cv2.imread('src/162E_Poop_Detection/Detect_Poop/learning_node/learning_node/sample3.png')

# Convert the image to grayscale and apply edge detection
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 100, 200)

# Find the contour of the poop shape
contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
contour = max(contours, key=cv2.contourArea)

# Store the poop shape as a numpy array
poop_shape = np.array(contour)
print(poop_shape)

# Create a blank white image with the same dimensions as the original image
height, width, _ = img.shape
blank_image = np.zeros((height, width, 3), np.uint8)
blank_image.fill(255)

# Draw the contour of the poop_shape on the blank white image
cv2.drawContours(blank_image, [poop_shape], 0, (0, 0, 0), 2)

# Display the resulting image
cv2.imshow('Poop Shape', blank_image)

# # Draw the contour on the original image
# cv2.drawContours(img, [contour], -1, (0, 255, 0), 2)

# Show the image with the poop shape contour
cv2.imshow('poop', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

