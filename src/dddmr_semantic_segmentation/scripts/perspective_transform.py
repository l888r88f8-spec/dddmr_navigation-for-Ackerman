
# import necessary libraries 
 
import cv2 
import numpy as np 
import matplotlib.pyplot as plt
'''
# Turn on Laptop's webcam
cap = cv2.VideoCapture(0)
 
while True:
     
    ret, frame = cap.read()
 
    # Locate points of the documents
    # or object which you want to transform
    pts1 = np.float32([[0, 260], [640, 260],
                       [0, 400], [640, 400]])
    pts2 = np.float32([[0, 0], [400, 0],
                       [0, 640], [400, 640]])
     
    # Apply Perspective Transform Algorithm
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    result = cv2.warpPerspective(frame, matrix, (500, 600))
     
    # Wrap the transformed image
    cv2.imshow('frame', frame) # Initial Capture
    cv2.imshow('frame1', result) # Transformed Capture
 
    if cv2.waitKey(24) == 27:
        break
 
cap.release()
cv2.destroyAllWindows()
'''

# Load the image
img = cv2.imread('camera_center.png') 
 
# Create a copy of the image
img_copy = np.copy(img)
 
# Convert to RGB so as to display via matplotlib
# Using Matplotlib we can easily find the coordinates
# of the 4 points that is essential for finding the 
# transformation matrix
while True:
    img_copy = cv2.cvtColor(img_copy,cv2.COLOR_BGR2RGB)
    pts1 = np.float32([[409, 484], [878, 488],
                       [1273, 646], [0, 638]])
    scale = 5
    pts2 = np.float32([[0, 0], [200*scale, 0],
                       [200*scale, 190*scale], [0, 190*scale]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    result = cv2.warpPerspective(img_copy, matrix, (200*scale, 190*scale))
    # Apply the perspective transformation to the image
    #result = cv2.warpPerspective(img_copy, matrix, (img_copy.shape[1], img_copy.shape[0]),flags=cv2.INTER_LINEAR)
    #cv2.imshow('camera_center', img_copy)
    #plt.imshow(result)
    #plt.show()
    cv2.imshow('camera_perspective', result)
    if cv2.waitKey(1) == ord('q'):
        break
cv2.destroyAllWindows()        