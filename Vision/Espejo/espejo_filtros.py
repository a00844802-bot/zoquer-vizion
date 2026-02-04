import sensor
import time
import pyb
import math
from pyb import UART

# Comunicación UART
UART_PORT = 3
UART_BAUDRATE = 115200

# Formato: (L_min, L_max, A_min, A_max, B_min, B_max)
THRESHOLD_BALL = (23, 93, 20, 83, 87, -4)      # Pelota (naranja/rojo)
THRESHOLD_OWN_GOAL = (12, 18, 5, -9, -14, -3)  # Portería propia (azul)
THRESHOLD_RIVAL_GOAL = (30, 63, -11, 8, 38, 15) # Portería rival (amarillo)

# Centro de referencia del robot (Ajustar al centro del espejo)
X_CENTER = 144
Y_CENTER = 148

# Parámetros de detección
BALL_AREA_THRESHOLD = 20        # Área mínima para detectar pelota
GOAL_AREA_THRESHOLD = 200       # Área mínima para detectar porterías
GOAL_PIXELS_THRESHOLD = 100     # Píxeles mínimos para porterías

# Detección de pelota mejorada
AREA_MINIMA_CERCA = 100
AREA_MINIMA_LEJOS = 4

# Zonas muertas para evitar vibraciones (en grados)
BALL_DEAD_ZONE = 5.0
RIVAL_GOAL_DEAD_ZONE = 20.0
OWN_GOAL_DEAD_ZONE = 5.0

# Ajustes de distancia cuando objeto está muy atrás (>135° o <-135°)
BALL_DISTANCE_CORRECTION = 10.0
OWN_GOAL_DISTANCE_CORRECTION = -30.0
OWN_GOAL_MIN_DISTANCE = 10.0

# Configuración de cámara
CAMERA_EXPOSURE = 25000  
CAMERA_GAIN_CEILING = 16
CAMERA_BRIGHTNESS = 1
CAMERA_CONTRAST = -5
CAMERA_SATURATION = -6

# Delay entre lecturas 
LOOP_DELAY = 50

# INICIALIZACIÓN 

def initialize_camera():

    led = pyb.LED(1)
    led.on()
    time.sleep(1)
    led.off()
    
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)  # 320x240
    
    print(" INICIALIZANDO CÁMARA ")
    print("Gain Ceiling:", sensor.get_gain_db())
    print("Exposure:", sensor.get_exposure_us())
    
    sensor.skip_frames(time=500)
    
    sensor.set_auto_gain(False)
    sensor.set_gainceiling(CAMERA_GAIN_CEILING)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=CAMERA_EXPOSURE)
    sensor.set_brightness(CAMERA_BRIGHTNESS)
    sensor.set_contrast(CAMERA_CONTRAST)
    sensor.set_saturation(CAMERA_SATURATION)
    
    sensor.set_hmirror(True)
    sensor.set_vflip(False)
    sensor.set_transpose(True)

def initialize_uart():

    uart = UART(UART_PORT, UART_BAUDRATE, timeout_char=0)
    uart.init(UART_BAUDRATE, bits=8, parity=None, stop=1)
    print("UART configurado en puerto", UART_PORT, "a", UART_BAUDRATE, "baudios\n")
    return uart

# DETECCIÓN DE OBJETOS 

def find_ball(img):

    blobs = img.find_blobs(
        [THRESHOLD_BALL], 
        area_threshold=BALL_AREA_THRESHOLD, 
        merge=True
    )
    
    valid_blobs = []
    
    if blobs:
        # Priorizar blobs grandes y cercanos al centro
        blobs = sorted(
            blobs,
            key=lambda b: (b.area(), -calculate_distance(b)),
            reverse=True
        )
        
        for blob in blobs:
            area = blob.area()
            circularidad = blob.compactness()
            
            # Pelota cerca
            if area >= AREA_MINIMA_CERCA and circularidad > 0.3:
                valido = True
            # Pelota lejos
            elif AREA_MINIMA_LEJOS <= area < AREA_MINIMA_CERCA:
                valido = True
            else:
                valido = False
            
            if valido:
            
                img.draw_rectangle(blob.rect(), color=(255, 255, 255))
                img.draw_cross(blob.cx(), blob.cy(), color=(255, 255, 255))

                if area >= AREA_MINIMA_CERCA:
                    radio = int((blob.w() + blob.h()) / 4)
                    img.draw_circle(blob.cx(), blob.cy(), radio, color=(255, 255, 255))
                
                valid_blobs.append(blob)
                break  
    
    return valid_blobs

def find_rival_goal(img):

    blobs = img.find_blobs(
        [THRESHOLD_RIVAL_GOAL],
        pixels_threshold=GOAL_PIXELS_THRESHOLD,
        area_threshold=GOAL_AREA_THRESHOLD,
        merge=True
    )
    
    for blob in blobs:
        img.draw_rectangle(blob.rect(), color=(0, 255, 0))
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
    
    return blobs

def find_own_goal(img):

    blobs = img.find_blobs(
        [THRESHOLD_OWN_GOAL],
        pixels_threshold=GOAL_PIXELS_THRESHOLD,
        area_threshold=GOAL_AREA_THRESHOLD,
        merge=True
    )
    
    for blob in blobs:
        img.draw_rectangle(blob.rect(), color=(0, 0, 255))
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 0, 255))
    
    return blobs

# CÁLCULOS GEOMÉTRICOS 

def calculate_distance(blob):

    dx = blob.cx() - X_CENTER
    dy = blob.cy() - Y_CENTER
    return math.sqrt(dx**2 + dy**2)

def calculate_angle(blob):

    dx = blob.cx() - X_CENTER
    dy = blob.cy() - Y_CENTER
    
    angle_rad = math.atan2(dy, dx)
    angle_deg = math.degrees(angle_rad)
    
    if angle_deg < 0.0:
        angle_deg = 360.0 + angle_deg
    
    angle_adjusted = -(angle_deg - 180.0)
    
    return angle_adjusted

def apply_dead_zone(angle, dead_zone):

    if -dead_zone < angle < dead_zone:
        return 0.0
    return angle

def correct_distance(distance, angle, correction, threshold_angle=135.0):

    if angle > threshold_angle or angle < -threshold_angle:
        return distance + correction
    return distance

# LOOP PRINCIPAL 

def main():

    initialize_camera()
    uart = initialize_uart()
    
    ball_distance = 0.0
    ball_angle = 0.0
    rival_goal_distance = 0.0
    rival_goal_angle = 0.0
    own_goal_distance = 0.0
    own_goal_angle = 0.0
    
    clock = time.clock()
    
    print(" INICIANDO DETECCIÓN \n")
    
    while True:
        clock.tick()
        img = sensor.snapshot()
        
        img.draw_cross(X_CENTER, Y_CENTER, color=(255, 255, 255))
        
        # DETECCIÓN DE PELOTA 
        ball_blobs = find_ball(img)
        if ball_blobs:
            blob = ball_blobs[0]
            ball_distance = calculate_distance(blob)
            ball_angle = calculate_angle(blob)
            
            ball_angle = apply_dead_zone(ball_angle, BALL_DEAD_ZONE)
            
            ball_distance = correct_distance(
                ball_distance, 
                ball_angle, 
                BALL_DISTANCE_CORRECTION
            )
        else:
            ball_distance = 0.0
            ball_angle = 0.0
        
        # DETECCIÓN DE PORTERÍA RIVAL 
        rival_goal_blobs = find_rival_goal(img)
        if rival_goal_blobs:
            blob = rival_goal_blobs[0]
            rival_goal_distance = calculate_distance(blob)
            rival_goal_angle = calculate_angle(blob)
            
            rival_goal_angle = apply_dead_zone(
                rival_goal_angle, 
                RIVAL_GOAL_DEAD_ZONE
            )
        else:
            rival_goal_distance = 0.0
            rival_goal_angle = 0.0
        
        # DETECCIÓN DE PORTERÍA PROPIA 
        own_goal_blobs = find_own_goal(img)
        if own_goal_blobs:
            blob = own_goal_blobs[0]
            own_goal_distance = calculate_distance(blob)
            own_goal_angle = calculate_angle(blob)
            
            own_goal_angle = apply_dead_zone(own_goal_angle, OWN_GOAL_DEAD_ZONE)
            
            own_goal_distance = correct_distance(
                own_goal_distance,
                own_goal_angle,
                OWN_GOAL_DISTANCE_CORRECTION
            )
            
            if own_goal_distance < OWN_GOAL_MIN_DISTANCE:
                own_goal_distance = OWN_GOAL_MIN_DISTANCE
        else:
            own_goal_distance = 0.0
            own_goal_angle = 0.0
        
        # ENVIAR DATOS POR UART 
        data = "{:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f}\n".format(
            ball_distance,
            ball_angle,
            rival_goal_distance,
            rival_goal_angle,
            own_goal_distance,
            own_goal_angle
        )
        
        print("Enviando:", data.strip())
        uart.write(data)
        
        pyb.delay(LOOP_DELAY)

if __name__ == "__main__":
    main()