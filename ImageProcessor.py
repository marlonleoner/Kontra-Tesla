import cv2
import numpy as np
import logging
import math
import datetime
import sys

class ImageProcessor():

    def __init__(self):
        self.mask = ''
        self.edges = ''
        self.cropped_edges = ''
        self.lane_lines_image = ''

    def ProcessImage(self, frame):
        #Função principal
        self.edges = self.detect_edges(frame)
        self.cropped_edges = self.region_of_interest(self.edges)

        line_segments = self.detect_line_segments(self.cropped_edges)

        lane_lines = self.average_slope_intercept(frame, line_segments)
        self.lane_lines_image = self.display_lines(frame, lane_lines)

        return lane_lines, self.lane_lines_image

    def detect_edges(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # TESTAR
        #filtra luz
        lower_color_bound = np.array([60, 40, 0])
        upper_color_bound = np.array([150, 255, 255])
        self.mask = cv2.inRange(hsv, lower_color_bound, upper_color_bound)

        # detecta as bordas
        edges = cv2.Canny(self.mask, 200, 400)

        return edges

    def region_of_interest(self, edges):
        height, width = edges.shape
        mask = np.zeros_like(edges)

        # TESTAR
        # Corta metade da imagem
        polygon = np.array([[
            (0, height * 1 / 2),
            (width, height * 1 / 2),
            (width, height),
            (0, height),
        ]], np.int32)

        cv2.fillPoly(mask, polygon, 255)
        masked_image = cv2.bitwise_and(edges, mask)

        return masked_image

    def detect_line_segments(self, cropped_edges):
        # TESTAR
        precision = 2  # precisão de 1 pixel
        angle = np.pi / 180  # 1 grau em radiano
        min_threshold = 20  # Minimo de votos para reconhecer como reta
        line_segments = cv2.HoughLinesP(cropped_edges, precision, angle, min_threshold, np.array([]), minLineLength=8,
                                        maxLineGap=4)

        return line_segments

    def average_slope_intercept(self, frame, line_segments):
        lane_lines = []
        if line_segments is None:
            print('Nenhum segmento detectado')
            return lane_lines

        height, width, _ = frame.shape
        left_fit = []
        right_fit = []

        # TESTAR
        boundary = 1/3
        left_region_boundary = width * (1 - boundary)  # Linha esquerda tem q estar no 2/3 da esquerda da tela
        right_region_boundary = width * boundary # Linha direita tem q estar no 2/3 da direita da tela

        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:
                    continue
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:
                    if x1 < left_region_boundary and x2 < left_region_boundary:
                        left_fit.append((slope, intercept))
                else:
                    if x1 > right_region_boundary and x2 > right_region_boundary:
                        right_fit.append((slope, intercept))

        left_fit_average = np.average(left_fit, axis=0)
        if len(left_fit) > 0:
            lane_lines.append(self.make_points(frame, left_fit_average))

        right_fit_average = np.average(right_fit, axis=0)
        if len(right_fit) > 0:
            lane_lines.append(self.make_points(frame, right_fit_average))

        return lane_lines

    def display_lines(self, frame, lines, line_color=(0, 255, 0), line_width=10):
        line_image = np.zeros_like(frame)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
        line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        return line_image

    def make_points(self, frame, line):
        height, width, _ = frame.shape
        slope, intercept = line
        y1 = height  # bottom of the frame
        # TESTAR
        y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

        # bound the coordinates within the frame
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]
