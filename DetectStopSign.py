import cv2

class DetectStopSign():

    def detect(self, cascade, grayImage, image):

        return cascade.detectMultiScale(
            grayImage,
            scaleFactor  = 1.1,
            minNeighbors = 5,
            minSize      = (30, 30))