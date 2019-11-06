import cv2
import math
import numpy

class DetectLines():

    def __init__(self, wImage = 320, hImage = 240):
        self.LLine  = []
        self.RLine  = []
        self.Line   = []
        self.wImage = wImage
        self.hImage = hImage

    def GetGray(self, image):
        # Convert to grayscale here.
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        return gray

    def GetCanny(self, image, Threshold1 = 100, Threshold2 = 200):
        # Call Canny Edge Detection here.
        canny = cv2.Canny(image, Threshold1, Threshold2)

        return canny

    def GetRegionOfInterest(self):
        x1, y1 = (1 * self.hImage / 8, self.wImage)
        x2, y2 = (1 * self.hImage / 2, self.wImage / 2)
        x3, y3 = (7 * self.hImage / 8, self.wImage)

        # x1, y1 = (0,          wImage)
        # x2, y2 = (hImage / 2, wImage / 2.5)
        # x3, y3 = (hImage,     wImage)

        verticesRegionOfInterest = [
            (x1, y1),
            (x2, y2),
            (x3, y3),
        ]

        return verticesRegionOfInterest

    def GetCrop(self, image, vertices):
        mask = numpy.zeros_like(image)

        matchMaskColor = 255

        cv2.fillPoly(mask, vertices, matchMaskColor)

        return cv2.bitwise_and(image, mask)

    def GetHoughLines(self, image, Rho = 5, Theta = numpy.pi / 180, Threshold = 120, MaxLineLen = 10, MaxLineGap = 50):
        return cv2.HoughLinesP(
            image,
            rho           = Rho,
            theta         = Theta,
            threshold     = Threshold,
            lines         = numpy.array([]),
            minLineLength = MaxLineLen,
            maxLineGap    = MaxLineGap
        )

    def MakeCoordinates(self, image, params):
        # Slope and Intersection
        slope, inter = params

        y1 = image.shape[0]
        y2 = int(y1 * (3 / 5))

        x1 = int((y1 - inter) / slope)
        x2 = int((y2 - inter) / slope)

        return numpy.array([x1, y1, x2, y2])

    def ProcessLines(self, image, lines):
        LFit = []
        RFit = []

        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)

            params = numpy.polyfit((x1, x2), (y1, y2), 1)

            slope = params[0]
            inter = params[1]

            if slope < 0:
                LFit.append((slope, inter))
            else:
                RFit.append((slope, inter))

        if len(LFit) == 0:
            LFit.append([-1.0, 350])
        if len(RFit) == 0:
            RFit.append([1.5, -250])

        LLinesAverage = numpy.average(LFit, axis = 0)
        RLinesAverage = numpy.average(RFit, axis = 0)

        self.LLine = self.MakeCoordinates(image, LLinesAverage)
        self.RLine = self.MakeCoordinates(image, RLinesAverage)

    def ProcessDirection(self):
        # Line Direction
        self.Line = [
            int(self.hImage / 2),
            int(self.hImage),
            int((self.LLine[2] + self.RLine[2]) / 2),
            int((self.LLine[3] + self.RLine[3]) / 2)
        ]

    def ProcessFrame(self, frame):
        image = numpy.copy(frame)

        # Grayscale
        gray = self.GetGray(image)

        # Canny
        canny = self.GetCanny(gray)

        # Vertices
        vertices = self.GetRegionOfInterest()

        # Moved the cropping operation to the end of the pipeline.
        crop = self.GetCrop(
            canny,
            numpy.array([vertices], numpy.int32)
        )

        # Get lines from Hough Transformation
        lines = self.GetHoughLines(crop)

        # Find Lines -> Left and Right
        self.ProcessLines(image, lines)

        # Find Line Direction
        self.ProcessDirection()

    def DrawLinesInImage(self, image):
        image = numpy.copy(image)

        if self.LLine in not None:
            # Left Line Coord
            LX1 = self.LLine[0]
            LY1 = self.LLine[1]
            LX2 = self.LLine[2]
            LY2 = self.LLine[3]

            # Draw Left Line
            cv2.line(image, (LX1, LY1), (LX2, LY2), [0, 0, 255], 3)

        if self.RLine in not None:
            # Right Line Coord
            RX1 = self.RLine[0]
            RY1 = self.RLine[1]
            RX2 = self.RLine[2]
            RY2 = self.RLine[3]

            # Draw Right Line
            cv2.line(image, (RX1, RY1), (RX2, RY2), [0, 255, 0], 3)

        if self.Line in not None:
            # Line Coord
            X1 = self.Line[0]
            Y1 = self.Line[1]
            X2 = self.Line[2]
            Y2 = self.Line[3]

            # Draw Line
            cv2.line(image, (X1, Y1), (X2, Y2), [255, 0, 0], 3)

        # Return the modified image
        return image
