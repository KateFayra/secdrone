# USAGE
# python detect.py --images images

# import the necessary packages
from __future__ import print_function
from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
import argparse
import imutils
import cv2

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--images", required=True, help="path to images directory")
args = vars(ap.parse_args())

# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# loop over the image paths
imagePaths = list(paths.list_images(args["images"]))

totalBeforeSup = 0
totalAfterSup = 0

for imagePath in imagePaths:
	# load the image and resize it to (1) reduce detection time
	# and (2) improve detection accuracy
	image = cv2.imread(imagePath)

	# detect people in the image
	(rects, weights) = hog.detectMultiScale(image, winStride=(2, 2), padding=(8, 8), scale=4.2)
		
	if len(rects) == 0:
		print("Did not find any in initial window")
		image = imutils.resize(image, width=min(400, image.shape[1]))
		(rects, weights) = hog.detectMultiScale(image, winStride=(4, 4), padding=(8, 8), scale=1.05)
		
	orig = image.copy()

	# draw the original bounding boxes
	for (x, y, w, h) in rects:
		cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)

	# apply non-maxima suppression to the bounding boxes using a
	# fairly large overlap threshold to try to maintain overlapping
	# boxes that are still people
	rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
	pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

	# draw the final bounding boxes
	for (xA, yA, xB, yB) in pick:
		cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)

	# show some information on the number of bounding boxes
	filename = imagePath[imagePath.rfind("/") + 1:]
	print("[INFO] {}: {} original boxes".format(
		filename, len(rects)))
		
	if len(rects) > 0:
		totalBeforeSup = totalBeforeSup + 1
		
	if len(pick) > 0:
		totalAfterSup = totalAfterSup + 1
		
	print("totalBeforeSup: " + str(totalBeforeSup))
	print("weights: " + str(weights))
	print("totalAfterSup: " + str(totalAfterSup))

	# show the output images
	#cv2.imshow("Before NMS", orig)
	#cv2.imshow("After NMS", image)
	#cv2.waitKey(0)
	
	cv2.imwrite("imagesout/" + filename + "." + str(weights) + ".jpg", orig)
	cv2.imwrite("imagesoutafter/" + filename + "." + str(weights) + ".jpg", image)
	
	
print("totalBeforeSup: " + str(totalBeforeSup))
#print("totalAfterSup: " + str(totalAfterSup))