import io
import cv2
import time
import numpy

from imutils.video import WebcamVideoStream
from imutils       import opencv2matplotlib
from PIL           import Image

import paho.mqtt.client as mqtt

# MQTT_BROKER = "10.0.1.15"
MQTT_BROKER = "localhost"
# MQTT_PORT   = 1883
MQTT_PORT   = 17090
MQTT_QOS    = 1

MQTT_TOPIC = "c0/s/c/0"

FPS = 1

def imageToByteArray(image):
    imgByteArray = io.BytesIO()
    image.save(imgByteArray, "PNG")

    return imgByteArray.getvalue()

# Callback da conexao
def onConnect(client, userdata, flags, rc):
    if rc == 0:
        print(" > [StreamFrames] Connected!")
    else:
        print(" > [StreamFrames] Bad Connection! Error code: " + str(rc))

def getMQTTClient():
    client = mqtt.Client()
    client.on_connect = onConnect

    return client

def main():
    client = getMQTTClient()
    client.connect(MQTT_BROKER, port=MQTT_PORT)

    # Aguarda o setup do MQTT
    time.sleep(4)

    # Loop
    client.loop_start()

    # Abre a camera
    camera = cv2.VideoCapture(0)
    time.sleep(2)

    while True:
        # Capture frame-by-frame
        ret, frame = camera.read()

        np_array_RGB = opencv2matplotlib(frame)  # Convert to RGB

        image = Image.fromarray(np_array_RGB)  #  PIL image
        byte_array = imageToByteArray(image)
        client.publish(MQTT_TOPIC, byte_array, qos = MQTT_QOS)

        print(" > [StreamFrames Published frame!")

        time.sleep(1 / FPS)

    # When everything done, release the capture
    camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
