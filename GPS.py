import cv2
import numpy as np
import logging
import math
import datetime
import sys

class GPS():

    def ProcessCommands(self, frame, lane_lines, current_angle):
        if len(lane_lines) == 0:
            print('Não foi detectado caminho')
            return frame

        new_steering_angle = self.compute_steering_angle(frame, lane_lines)
        current_angle = self.stabilize_steering_angle(current_angle, new_steering_angle, len(lane_lines))

        print('Current steering angle: ', current_angle)

        curr_heading_image = self.add_heading_line(frame, current_angle)

        return current_angle, curr_heading_image


    def compute_steering_angle(self, frame, lane_lines):
        #Acha o angulo baseado nas coordenadas dos caminhos
        if len(lane_lines) == 0:
            print('Não foi detectado caminho')
            return -90

        height, width, _ = frame.shape
        if len(lane_lines) == 1:
            # Detectado só um lado, seguir ele
            x1, _, x2, _ = lane_lines[0][0]
            x_offset = x2 - x1
        else:
            _, _, left_x2, _ = lane_lines[0][0]
            _, _, right_x2, _ = lane_lines[1][0]
            camera_mid_offset_percent = 0.02
            mid = int(width / 2 * (1 + camera_mid_offset_percent))
            x_offset = (left_x2 + right_x2) / 2 - mid


        # TESTAR
        y_offset = int(height * 2/4)

        angle_to_mid_radian = math.atan(x_offset / y_offset)  # angulo em radianos para a linha central imaginaria
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # converte pra graus
        steering_angle = angle_to_mid_deg + 90  #  calcula o angulo

        print('Novo angulo: ', steering_angle)

        return steering_angle

    def stabilize_steering_angle(self, current_angle, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=1):
        if num_of_lane_lines == 2 :
            # achou os dois lados, pode desviar mais
            max_angle_deviation = max_angle_deviation_two_lines
        else :
            # achou só um lado, não desviar muito
            max_angle_deviation = max_angle_deviation_one_lane

        angle_deviation = new_steering_angle - current_angle
        if abs(angle_deviation) > max_angle_deviation:
            stabilized_steering_angle = int(current_angle
                                            + max_angle_deviation * angle_deviation / abs(angle_deviation))
        else:
            stabilized_steering_angle = new_steering_angle
        print('Proposed angle: ', new_steering_angle, 'stabilized angle: ', stabilized_steering_angle)

        return stabilized_steering_angle

    def add_heading_line(self, frame, steering_angle, line_color=(0, 0, 255), line_width=5, ):
        heading_image = np.zeros_like(frame)
        height, width, _ = frame.shape

        # Se o angulo estiver
        # 0-89 degree: virar a esquerda
        # 90 degree: indo reto
        # 91-180 degree: virar a direita
        steering_angle_radian = steering_angle / 180.0 * math.pi
        x1 = int(width / 2)
        y1 = height
        x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
        y2 = int(height / 2)

        cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
        heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

        return heading_image
