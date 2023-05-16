import cv2
import numpy as np
from matplotlib import pyplot as plt

# initialize the list to store the HSV values
hsv_values = []

# define a function to display the coordinates of
# the points clicked on the image
def click_event(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Extract pixel color
        color = img[y, x]

        # Convert BGR to HSV
        bgr_color = np.uint8([[[color[2], color[1], color[0]]]])
        hsv_color = cv2.cvtColor(bgr_color, cv2.COLOR_BGR2HSV)

        # Append the HSV values to the list
        hsv_values.append(hsv_color[0][0])

        # Print the HSV values
        print(f"HSV values: {hsv_values}")
        print(f"Average HSV value: {np.mean(hsv_values, axis=0)}")
        print(f"Lowest HSV value: {np.min(hsv_values, axis=0)}")
        print(f"Highest HSV value: {np.max(hsv_values, axis=0)}")

        # Draw a red dot on the clicked point
        cv2.circle(img, (x, y), 3, (0, 0, 255), -1)

        # Display the image with the clicked point
        cv2.imshow('Point Coordinates',img)

# read the input image
img = cv2.imread("src/162E_Poop_Detection/Detect_Poop/learning_node/learning_node/sample.jpg")

# create a window
cv2.namedWindow('Point Coordinates')

# bind the callback function to window
cv2.setMouseCallback('Point Coordinates', click_event)

# display the image using matplotlib
plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
plt.show()

# display the image using OpenCV
while True:
    cv2.imshow('Point Coordinates',img)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
cv2.destroyAllWindows()
