#pelota roja con base a la forma y el color

import math
import pyb
import sensor, image, time
from pyb import UART, LED

uart = UART(3, 115200, timeout_char=0)
uart.init(115200, bits=8, parity=None, stop=1)

# --- LEDs INDICADORES ---
led_rojo = LED(1)    # LED rojo
led_verde = LED(2)   # LED verde
led_azul = LED(3)    # LED azul

# --- UMBRAL DE COLOR ROJO EN LAB ---
# Formato: (L Min, L Max, A Min, A Max, B Min, B Max)
# Para rojo: A alto (positivo indica rojo)
red_threshold = (30, 100, 15, 127, 15, 127)
blue_threshold = (0, 35, 33, -10, -122, -14)
yellow_threshold = (50, 73, -55, 48, 8, 28)

def initialize_camera():
    # --- CONFIGURACIÓN DE CÁMARA ---
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)  # 320x240
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)

# --- PARÁMETROS ---
ANCHO = 320
ALTO = 240

ZONA_MUERTA = 50
AREA_MINIMA = 100
RADIO_MINIMO = 20

CENTRO_X = ANCHO // 2
CENTRO_Y = ALTO // 2

clock = time.clock()

def find_red_ball(img):
    blob_ball = img.find_blobs([red_threshold], pixels_threshold=200,
                               area_threshold=AREA_MINIMA, merge=True)

    if not blob_ball:
        return None

    # Ordenar por área (mayor primero)
    blobs = sorted(blob_ball, key=lambda b: b.area(), reverse=True)

    for blob in blobs:
        # Calcular aspect ratio
        aspect_ratio = blob.w() / float(blob.h()) if blob.h() > 0 else 0

        # DETECCIÓN DE PELOTA
        circularidad = blob.compactness()

        if circularidad > 0.5: 
            img.draw_rectangle(blob.rect(), color=(0, 255, 0))
            img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))

            desp_x = blob.cx() - CENTRO_X
            desp_y = blob.cy() - CENTRO_Y

            radio = (blob.w() + blob.h()) / 4

            img.draw_string(10, 10, "R:%d X:%d" % (int(radio), desp_x),
                            color=(255, 255, 255), scale=1)

    return blob_ball

def find_rival_goal(img):
    blob_goal = img.find_blobs([blue_threshold], pixels_threshold=200,
                               area_threshold=100, merge=True)

    if not blob_goal:
        return None

    for blob in blob_goal:
        img.draw_rectangle(blob.rect(), color=(0, 0, 255))
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 0, 255))
        print("Rival goal detected at: ", blob.cx(), blob.cy())

    return blob_goal

def find_friendly_goal(img):
    blob_goal = img.find_blobs([blue_threshold], pixels_threshold=200,
                               area_threshold=100, merge=True)

    if not blob_goal:
        return None

    for blob in blob_goal:
        img.draw_rectangle(blob.rect(), color=(0, 0, 255))
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 0, 255))
        print("Friendly goal detected at: ", blob.cx(), blob.cy())

    return blob_goal

def distance(blob):
    relative_distx = blob.cx() - CENTRO_X
    relative_disty = blob.cy() - CENTRO_Y

    vector_dist = math.sqrt(relative_distx**2 + relative_disty**2)
    distance = vector_dist
    return distance

def angle(blob):
    relative_distx = blob.cx() - CENTRO_X
    relative_disty = blob.cy() - CENTRO_Y
    print("Relative Distx: %d" % relative_distx)
    print("Relative Disty: %d" % relative_disty)

    angle = math.atan2(relative_disty, relative_distx)
    angle_degree = math.degrees(angle)
    if angle_degree < 0:
        angle_degree = 360 + angle_degree
    return angle_degree

def main():
    initialize_camera()
    global distance_b, distance_g, distance_gop, angle_ball, angle_goal, angle_gop

    clock = time.clock()

    distance_b = 0
    distance_g = 0
    angle_ball = 0
    angle_goal = 0
    distance_gop = 0
    angle_gop = 0
    pelota_detectada = False

    while True:
        clock.tick()
        img = sensor.snapshot()
        blob_ball = find_red_ball(img)
        blob_rgoal = find_rival_goal(img)
        blob_fgoal = find_friendly_goal(img)

        # If a red ball is detected
        if blob_ball:
            pelota_detectada = True

            for blob in blob_ball:
                distance_b = distance(blob)
                angle_ball = -(angle(blob) - 180)

                if angle_ball < 5 and angle_ball > -7:
                    angle_ball = 0

                elif distance_b < 30 and distance_b > -30:
                    angle_ball = 0
                    print("Distance Ball: %d" % distance_b)
                    print("Angle Ball: %d" % angle_ball)

        elif not blob_ball:
            distance_b = 0
            angle_ball = 0
            img.draw_string(10, 10, "BUSCANDO...", color=(255, 255, 0), scale=2)
            print("Buscando pelota...")
            pelota_detectada = False

        # If a rival goal is detected
        if blob_rgoal:
            for blob in blob_rgoal:
                distance_g = distance(blob)
                angle_goal = -(angle(blob) - 180)

                if angle_goal < 5 and angle_goal > -7:
                    angle_goal = 0
                    distance_g = 0

        elif not blob_rgoal:
            distance_g = 0
            angle_goal = 0

        # If a friendly goal is detected
        if blob_fgoal:
            for blob in blob_fgoal:
                distance_gop = distance(blob)
                angle_gop = -(angle(blob) - 180)

                if angle_gop < 5 and angle_gop > -7:
                    angle_gop = 0
                    distance_gop = 0

        elif not blob_fgoal:
            distance_gop = 0
            angle_gop = 0

        print("FPS: %d" % clock.fps())

        data = "{} {} {} {} {} {}\n".format(distance_b, angle_ball, distance_g, angle_goal, distance_gop, angle_gop)
        print("Sending: ", data)
        uart.write(data)
        pyb.delay(50)

main()