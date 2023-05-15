import numpy as np

# your data
camera_read = np.array([558, 613, 692, 767, 810, 895, 937, 983, 1015, 1048])
actual_distance = np.array([500, 550, 600, 650, 700, 750, 800, 850, 900, 950])

# # apply linear regression
# m, c = np.polyfit(camera_read, actual_distance, 1)

# print(f"Slope: {m}, Intercept: {c}")


# apply quadratic regression
a, b, c = np.polyfit(camera_read, actual_distance, 2) #computes a polynomial fit of degree 2.

print(f"a: {a}, b: {b}, c: {c}")
#Actual_Distance = a * (Camera_Read^2) + b * Camera_Read + c