import os
import io
import sys
import cv2
import time
import math
import numpy
import traceback

from PIL       import Image
from threading import Thread

import paho.mqtt.client as mqtt

from DetectStopSign import *
from DetectLines    import *

MQTT_BROKER = "10.0.1.15"
MQTT_PORT   = 1883
MQTT_QOS    = 1

MQTT_TOPIC = "c0/s/c/0"

global client

class MQTTClient():

    def __init__(self, broker, topic, port = MQTT_PORT):
        self.broker = broker
        self.port   = port
        self.topic  = topic

        self.gray  = None
        self.image = None

        self.client = mqtt.Client()
        self.client.on_connect = self.onConnect
        self.client.on_message = self.onMessage

    def connect(self):
        self.client.connect(self.broker, port = self.port)

    def disconnect(self):
        self.client.disconnect()

    def subscribe(self):
        self.client.subscribe(self.topic)

    def loop(self):
        self.client.loop_forever()

    # Callback da conexao
    def onConnect(self, client, userdata, flags, rc):
        if rc == 0:
            print(" > [GetFrames] Connected!")
        else:
            print(" > [GetFrames] Bad Connection! Error code: " + str(rc))

    # Callback das mensagens
    def onMessage(self, client, userdata, msg):
        print(" > [GetFrames] Frame received!")

        try:
            imageFromPayload = msg.payload

            self.gray  = cv2.imdecode(numpy.frombuffer(imageFromPayload, dtype=numpy.uint8), cv2.IMREAD_GRAYSCALE)
            self.image = cv2.imdecode(numpy.frombuffer(imageFromPayload, dtype=numpy.uint8), cv2.IMREAD_COLOR)

        except Exception as exc:
            print("Exception in user code:", exc)
            print('-' * 60)
            traceback.print_exc(file = sys.stdout)
            print('-' * 60)

class ShowFrames():

    detectLineObj     = DetectLines()
    detectStopSignObj = DetectStopSign()
    cascade           = cv2.CascadeClassifier("cascades/cascade.xml")

    def __init__(self, client):
        self.client   = client
        self.started  = False
        self.run      = True

    def start(self):
        print(" > [ShowFrames] Creating Thread ... ")
        if self.started :
            print(" > [ShowFrames] Thread Already Started!")
            return None

        self.started = True
        self.thread  = Thread(target = self.showImage, args = ())
        self.thread.start()
 
        return self

    def showImage(self):
        print(" > [ShowFrames] Thread Started")
        try:
            while(self.run):
                gray  = self.client.gray
                image = self.client.image
                
                if(image is not None):    
                    wImage = image.shape[0]
                    hImage = image.shape[1]

                    x1, y1 = (0,          wImage)
                    x2, y2 = (hImage / 2, wImage / 2)
                    x3, y3 = (hImage,     wImage)

                    verticesRegionOfInterest = [
                        (x1, y1),
                        (x2, y2),
                        (x3, y3),
                    ]

                    # Convert to grayscale here.
                    grayImage = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

                    # Call Canny Edge Detection here.
                    cannyedImage = cv2.Canny(grayImage, 100, 200)

                    # Moved the cropping operation to the end of the pipeline.
                    croppedImage = self.detectLineObj.cropImage(
                        cannyedImage,
                        numpy.array([verticesRegionOfInterest], numpy.int32)
                    )

                    cv2.imshow('Image', croppedImage)

                    lines = cv2.HoughLinesP(
                        croppedImage,
                        rho           = 10,
                        theta         = numpy.pi / 180,
                        threshold     = 160,
                        lines         = numpy.array([]),
                        minLineLength = 40,
                        maxLineGap    = 25
                    )

                    # print(" > [Script] Lines: " + str(lines))

                    xLeftLine  = []
                    yLeftLine  = []
                    xRightLine = []
                    yRightLine = []

                    if(lines is not None):
                        for line in lines:
                            for x1, y1, x2, y2 in line:
                                # The slope of a line is a number that describes both the direction and the steepness of the line
                                slope = (y2 - y1) / (x2 - x1)
                                
                                # Only consider theta > 26º
                                if math.fabs(slope) < 0.5:
                                    continue
                                
                                # Since the slope is negative, the direction of the line is decreasing => right line
                                if slope <= 0:
                                    xRightLine.extend([x1, x2])
                                    yRightLine.extend([y1, y2])
                                # Otherwise => left line.
                                else: 
                                    xLeftLine.extend([x1, x2])
                                    yLeftLine.extend([y1, y2])

                        if(len(xLeftLine) != 0 and len(yLeftLine) != 0 and len(xRightLine) != 0 and len(yRightLine) != 0):
                            minY = image.shape[0] * (6 / 10) # Just below the horizon
                            maxY = image.shape[0]           # The bottom of the image

                            polyLeft = numpy.poly1d(numpy.polyfit(
                                yLeftLine,
                                xLeftLine,
                                deg = 1
                            ))

                            leftStartX = int(polyLeft(maxY))
                            leftEndX   = int(polyLeft(minY))

                            polyRight = numpy.poly1d(numpy.polyfit(
                                yRightLine,
                                xRightLine,
                                deg = 1
                            ))

                            rightStartX = int(polyRight(maxY))
                            rightEndX   = int(polyRight(minY))

                            image = self.detectLineObj.drawLinesInImage(
                                image,
                                [[
                                    [leftStartX,  maxY, leftEndX,  int(minY)],
                                    [rightStartX, maxY, rightEndX, int(minY)],
                                ]],
                                thicknessLine = 5
                            )

                    # Identify STOP SIGN in frame
                    stopSignObjs = self.detectStopSignObj.detect(self.cascade, gray, image)
                    # Draw a rectangle around the objects found
                    for (x_pos, y_pos, width, height) in stopSignObjs:
                        x1, y1 = x_pos,         y_pos
                        x2, y2 = x_pos + width, y_pos + height
                        cv2.rectangle(image, (x1,     y1),     (x2,     y2),     (255, 255, 255), 2)
                        cv2.rectangle(image, (x1 + 3, y1 + 3), (x2 - 3, y2 - 3), (  0,   0, 255), 2)               

                    # Show Frame
                    cv2.imshow('Image', image)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print(" > [ShowFrames] Thread Stopped")
                    self.run = False

        except Exception as exc:
            print(" > [ShowFrames] showImage() Exception:", exc)
            print('-' * 60)
            traceback.print_exc(file = sys.stdout)
            print('-' * 60)

        finally:
            cv2.destroyAllWindows()
            self.stop()
            sys.exit()

    def stop(self):
        self.client.disconnect()

def main():
    client = MQTTClient(MQTT_BROKER, MQTT_TOPIC)
    client.connect()
    client.subscribe()

    # Aguarda o setup do MQTT
    time.sleep(4)

    # Video
    video = ShowFrames(client).start()

    # Inicia o loop
    client.loop()

if __name__ == "__main__":
    main()
