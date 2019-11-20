import cv2
import numpy as np
import ImageProcessor
import GPS

class Car():

    def __init__(self):
        self.imageProcessor = ImageProcessor.ImageProcessor()
        self.gps = GPS.GPS()
        self.current_angle = 90
        self.processedImage = ""
        self.turnCommand = tuple()

    def Drive(self, frame):
        self.frame = frame

        # Processa a imagem
        lane_lines, self.processedImage = self.imageProcessor.ProcessImage(frame)

        # Calcula o Angulo
        self.current_angle, self.processedImage = self.gps.ProcessCommands(self.processedImage, lane_lines, self.current_angle)

        # Calcula o quanto tem q virar
        self.Turn()

        # Manda o comando
        self.SendCommand()

    def SendCommand(self):
        print("mandando:", self.turnCommand)

    def Turn(self):
        anguloReto = 90
        maxVelocidade = 50

        angulo = self.current_angle
        diferencaAngulos = abs(anguloReto - angulo)  #diferença entre os angulos, o quanto tem que mover  abs=módulo
        if angulo == anguloReto:
            diferencaAngulos = angulo #se for o mesmo angulo, conserva

        if angulo < anguloReto:
            lado = "e" #vê de que lado tem que diminuir
        else:
            lado = "d"

        #regra de tres pra achar a porção que tem que diminuir, diminui da velocidade max, round pra arredondar
        novaVelocidade = round(maxVelocidade - (diferencaAngulos * maxVelocidade) / anguloReto)

        #retorna a nova velocidade de acordo com o lado que vira
        if lado == "e":
            self.turnCommand = (novaVelocidade, maxVelocidade)
        else:
            self.turnCommand = (maxVelocidade, novaVelocidade)

    def GetImage(self, index):
        if index == 1:
            return self.processedImage
        elif index == 2:
            return self.imageProcessor.mask
        elif index == 3:
            return self.imageProcessor.edges
        elif index == 4:
            return self.imageProcessor.cropped_edges
        else:
            return self.frame