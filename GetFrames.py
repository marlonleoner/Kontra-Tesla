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

# MQTT_BROKER = "10.0.1.15"
MQTT_BROKER = "localhost"
# MQTT_PORT   = 1883
MQTT_PORT   = 17090
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

    # detectLineObj     = DetectLines()
    # detectStopSignObj = DetectStopSign()
    car     = Car.Car()
    cascade = cv2.CascadeClassifier("cascades/cascade.xml")

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
                    # #
                    # self.detectLineObj.ProcessFrame(image)
                    # #
                    # image = self.detectLineObj.DrawLinesInImage(image)


                    car.Drive(frame)
        
                    frame = car.GetImage(1)


                    # # Identify STOP SIGN in frame
                    # stopSignObjs = self.detectStopSignObj.detect(self.cascade, gray, image)
                    # # Draw a rectangle around the objects found
                    # for (x_pos, y_pos, width, height) in stopSignObjs:
                    #     x1, y1 = x_pos,         y_pos
                    #     x2, y2 = x_pos + width, y_pos + height
                    #     cv2.rectangle(image, (x1,     y1),     (x2,     y2),     (255, 255, 255), 2)
                    #     cv2.rectangle(image, (x1 + 3, y1 + 3), (x2 - 3, y2 - 3), (  0,   0, 255), 2)

                    # Show Frame
                    cv2.imshow('Image', frame)

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
