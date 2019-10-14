import cv2
import numpy

class DetectLines():

    def cropImage(self, image, vertices):
        mask = numpy.zeros_like(image)

        matchMaskColor = 255

        cv2.fillPoly(mask, vertices, matchMaskColor)
        
        return cv2.bitwise_and(image, mask)

    def drawLinesInImage(self, image, lines, colorLine = [0, 0, 255], thicknessLine = 3):

        # If there are no lines to draw, exit.
        if lines is None:
            return

        # Make a copy of the original image.
        image = numpy.copy(image)

        # Create a blank image that matches the original in size.
        imageWithLines = numpy.zeros(
            (
                image.shape[0],
                image.shape[1],
                3
            ),
            dtype = numpy.uint8,
        )

        # Loop over all lines and draw them on the blank image.
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(imageWithLines, (x1, y1), (x2, y2), colorLine, thicknessLine)
        
        # Merge the image with the lines onto the original.
        image = cv2.addWeighted(image, 0.8, imageWithLines, 1.0, 0.0)
        
        # Return the modified image.
        return image
