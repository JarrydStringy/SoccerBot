# Imports
import cv2
import numpy as np

cap = cv2.VideoCapture(0)  # Connect to camera 0 (or the only camera)
cap.set(3, 320)  # Set the width to 320
cap.set(4, 240)  # Set the height to 240
# cap.set(3, 640)
# cap.set(4, 480)
ret, frame = cap.read()  # Get a frame from the camera

if ret:  # Check if data was obtained successfully
    cv2.imshow("CameraImage", frame)  # Display the obtained frame in a window called "CameraImage"
    cv2.waitKey(0)  # Make the program wait until you press a key before continuing.

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Convert from BGR to HSV colourspace)

    cv2.imshow("HSV_IMAGE", hsv_frame)
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_HSV2GRAY)

    ret, thresholded_frame = cv2.threshold(frame_blue, 127, 255, cv2.THRESH_BINARY)  # Threshold blue channel
    plt.imshow(gray, cmap='gray')
    cv2.imshow("Binary Thresholded Frame", thresholded_frame)  # Display thresholded frame
    cv2.waitKey(0)  # Exit on keypress

cap.release()  # Release the camera object
cv2.destroyAllWindows()  # Close all opencv pop-up windows


# ret, dst = cv2.threshold(src, thresh, maxValue, type)
    # dst 		→ 		The output image
    # src 		→ 		The input image (this should be a grayscale image)
    # thresh 	→ 		The threshold value
    # maxValue 	→ 		A maximum value, used for binary thresholding
    # type 		→ 		The thresholding method being used (see here for parameter options)

# cv2.imwrite("frame0001.png", frame)		                # Save the frame as frame0001.png
# lab_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab) 		# Convert from BGR to Lab colourspace
# b, g, r = cv2.split(frame)		                        # Split the frame into its 3 colour channels
# b = b*0					                                # Set the blue pixels to zero
# g = g*0					                                # Set the green pixels to zero
# frame = cv2.merge((b, g, r))		                        # Merge the channels back into one image
# frame[:, :, 0] = 0				                        # Set colour channel 0 (blue) to 0
# sub_frame = frame[200:300, 200:300, 0]		            # Extract blue colour channel of a 100 pixel region
# sub_frame = sub_frame * 2			                        # Double the pixel blue channel values
# frame[200:300, 200:300, 0] = sub_frame		            # Replace the region in the original image with our sub frame
