"""
Sistema de Visión Robot de Fútbol con Cámara a Espejo
Detecta: Pelota, Portería Rival, Portería Propia
Envía datos por UART: distancia y ángulo de cada objeto
"""

import sensor
import time
import pyb
import math
from pyb import UART

# CONFIGURACIÓN 

# Comunicación UART
UART_PORT = 3
UART_BAUDRATE = 115200

# Thresholds LAB para detección de colores
# Formato: (L_min, L_max, A_min, A_max, B_min, B_max)
THRESHOLD_BALL = (23, 93, 20, 83, 87, -4)      # Pelota (naranja/rojo)
THRESHOLD_OWN_GOAL = (12, 18, 5, -9, -14, -3)  # Portería propia (azul)
THRESHOLD_RIVAL_GOAL = (30, 63, -11, 8, 38, 15) # Portería rival (amarillo)

# Centro de referencia del robot (calibrar según tu cámara)
X_CENTER = 144
Y_CENTER = 148

# Parámetros de detección
BALL_AREA_THRESHOLD = 20        # Área mínima para detectar pelota
GOAL_AREA_THRESHOLD = 200       # Área mínima para detectar porterías
GOAL_PIXELS_THRESHOLD = 100     # Píxeles mínimos para porterías

# Zonas muertas para evitar vibraciones (en grados)
BALL_DEAD_ZONE = 5.0
RIVAL_GOAL_DEAD_ZONE = 20.0
OWN_GOAL_DEAD_ZONE = 5.0

# Ajustes de distancia cuando objeto está muy atrás (>135° o <-135°)
BALL_DISTANCE_CORRECTION = 10.0
OWN_GOAL_DISTANCE_CORRECTION = -30.0
OWN_GOAL_MIN_DISTANCE = 10.0

# Configuración de cámara
CAMERA_EXPOSURE = 25000  # Microsegundos (ajustar según iluminación)
CAMERA_GAIN_CEILING = 16
CAMERA_BRIGHTNESS = 1
CAMERA_CONTRAST = -5
CAMERA_SATURATION = -6

# Delay entre lecturas (milisegundos)
LOOP_DELAY = 50

# INICIALIZACIÓN 

def initialize_camera():
    """
    Configura la cámara OpenMV para detección óptima
    Enciende LED rojo durante 1 segundo para indicar inicio
    """
    # Indicador visual de inicio
    led = pyb.LED(1)  # LED Rojo
    led.on()
    time.sleep(1)
    led.off()
    
    # Reset y configuración básica
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)  # 320x240
    
    # Mostrar valores de calibración
    print("=== INICIALIZANDO CÁMARA ===")
    print("Gain Ceiling:", sensor.get_gain_db())
    print("Exposure:", sensor.get_exposure_us())
    
    # Esperar estabilización
    sensor.skip_frames(time=500)
    
    # Configuración manual para control total
    sensor.set_auto_gain(False)
    sensor.set_gainceiling(CAMERA_GAIN_CEILING)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=CAMERA_EXPOSURE)
    sensor.set_brightness(CAMERA_BRIGHTNESS)
    sensor.set_contrast(CAMERA_CONTRAST)
    sensor.set_saturation(CAMERA_SATURATION)
    
    # Orientación de imagen (ajustar según montaje)
    sensor.set_hmirror(True)   # Espejo horizontal
    sensor.set_vflip(False)    # Sin volteo vertical
    sensor.set_transpose(True) # Transponer imagen
    
    print("Cámara configurada correctamente\n")

def initialize_uart():
    """
    Configura comunicación UART para enviar datos al microcontrolador
    """
    uart = UART(UART_PORT, UART_BAUDRATE, timeout_char=0)
    uart.init(UART_BAUDRATE, bits=8, parity=None, stop=1)
    print("UART configurado en puerto", UART_PORT, "a", UART_BAUDRATE, "baudios\n")
    return uart

# DETECCIÓN DE OBJETOS 

def find_ball(img):
    """
    Detecta la pelota y la marca con rectángulo y cruz ROJA
    Returns: Lista de blobs detectados
    """
    blobs = img.find_blobs(
        [THRESHOLD_BALL], 
        area_threshold=BALL_AREA_THRESHOLD, 
        merge=True
    )
    
    for blob in blobs:
        img.draw_rectangle(blob.rect(), color=(255, 0, 0))  # Rojo
        img.draw_cross(blob.cx(), blob.cy(), color=(255, 0, 0))
    
    return blobs

def find_rival_goal(img):
    """
    Detecta la portería rival (amarilla) y la marca con rectángulo y cruz VERDE
    Returns: Lista de blobs detectados
    """
    blobs = img.find_blobs(
        [THRESHOLD_RIVAL_GOAL],
        pixels_threshold=GOAL_PIXELS_THRESHOLD,
        area_threshold=GOAL_AREA_THRESHOLD,
        merge=True
    )
    
    for blob in blobs:
        img.draw_rectangle(blob.rect(), color=(0, 255, 0))  # Verde
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
    
    return blobs

def find_own_goal(img):
    """
    Detecta la portería propia (azul) y la marca con rectángulo y cruz AZUL
    Returns: Lista de blobs detectados
    """
    blobs = img.find_blobs(
        [THRESHOLD_OWN_GOAL],
        pixels_threshold=GOAL_PIXELS_THRESHOLD,
        area_threshold=GOAL_AREA_THRESHOLD,
        merge=True
    )
    
    for blob in blobs:
        img.draw_rectangle(blob.rect(), color=(0, 0, 255))  # Azul
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 0, 255))
    
    return blobs

# CÁLCULOS GEOMÉTRICOS 

def calculate_distance(blob):
    """
    Calcula distancia euclidiana desde el centro hasta el blob (en píxeles)
    """
    dx = blob.cx() - X_CENTER
    dy = blob.cy() - Y_CENTER
    return math.sqrt(dx**2 + dy**2)

def calculate_angle(blob):
    """
    Calcula ángulo desde el centro hasta el blob
    Ángulo ajustado: 0° = frente del robot, positivo = derecha, negativo = izquierda
    Returns: Ángulo en grados (-180 a 180)
    """
    dx = blob.cx() - X_CENTER
    dy = blob.cy() - Y_CENTER
    
    # Calcular ángulo en radianes y convertir a grados
    angle_rad = math.atan2(dy, dx)
    angle_deg = math.degrees(angle_rad)
    
    # Normalizar a rango 0-360
    if angle_deg < 0.0:
        angle_deg = 360.0 + angle_deg
    
    # Ajustar para que 0° sea el frente del robot
    angle_adjusted = -(angle_deg - 180.0)
    
    return angle_adjusted

def apply_dead_zone(angle, dead_zone):
    """
    Aplica zona muerta: si el ángulo es muy pequeño, lo pone en 0
    Evita vibraciones innecesarias del robot
    """
    if -dead_zone < angle < dead_zone:
        return 0.0
    return angle

def correct_distance(distance, angle, correction, threshold_angle=135.0):
    """
    Corrige la distancia cuando el objeto está muy atrás (>135° o <-135°)
    """
    if angle > threshold_angle or angle < -threshold_angle:
        return distance + correction
    return distance

# LOOP PRINCIPAL 

def main():
    """
    Loop principal: detecta objetos, calcula posiciones y envía datos por UART
    """
    # Inicializar hardware
    initialize_camera()
    uart = initialize_uart()
    
    # Variables para almacenar datos
    ball_distance = 0.0
    ball_angle = 0.0
    rival_goal_distance = 0.0
    rival_goal_angle = 0.0
    own_goal_distance = 0.0
    own_goal_angle = 0.0
    
    clock = time.clock()
    
    print("=== INICIANDO DETECCIÓN ===\n")
    
    while True:
        clock.tick()
        img = sensor.snapshot()
        
        # Dibujar cruz blanca en el centro de referencia
        img.draw_cross(X_CENTER, Y_CENTER, color=(255, 255, 255))
        
        # DETECCIÓN DE PELOTA 
        ball_blobs = find_ball(img)
        if ball_blobs:
            blob = ball_blobs[0]  # Tomar el primer blob (el más grande)
            ball_distance = calculate_distance(blob)
            ball_angle = calculate_angle(blob)
            
            # Aplicar zona muerta
            ball_angle = apply_dead_zone(ball_angle, BALL_DEAD_ZONE)
            
            # Corregir distancia si está muy atrás
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
            
            # Zona muerta más amplia para portería
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
            
            # Aplicar zona muerta
            own_goal_angle = apply_dead_zone(own_goal_angle, OWN_GOAL_DEAD_ZONE)
            
            # Corregir distancia si está muy atrás
            own_goal_distance = correct_distance(
                own_goal_distance,
                own_goal_angle,
                OWN_GOAL_DISTANCE_CORRECTION
            )
            
            # Establecer distancia mínima
            if own_goal_distance < OWN_GOAL_MIN_DISTANCE:
                own_goal_distance = OWN_GOAL_MIN_DISTANCE
        else:
            own_goal_distance = 0.0
            own_goal_angle = 0.0
        
        # ===== MOSTRAR DATOS EN PANTALLA =====
        # Configuración de texto
        text_color = (255, 255, 255)  # Blanco
        text_scale = 1
        y_offset = 10  # Posición inicial Y
        
        # Título
        img.draw_string(5, y_offset, " DATOS ENVIADOS ", color=text_color, scale=text_scale)
        y_offset += 15
        
        # Datos de la pelota (ROJO)
        img.draw_string(5, y_offset, "PELOTA:", color=(255, 0, 0), scale=text_scale)
        y_offset += 12
        img.draw_string(5, y_offset, "Dist: %.1f px" % ball_distance, color=text_color, scale=text_scale)
        y_offset += 12
        img.draw_string(5, y_offset, "Ang: %.1f grados" % ball_angle, color=text_color, scale=text_scale)
        y_offset += 15
        
        # Datos de portería rival (VERDE)
        img.draw_string(5, y_offset, "PORTERIA RIVAL:", color=(0, 255, 0), scale=text_scale)
        y_offset += 12
        img.draw_string(5, y_offset, "Dist: %.1f px" % rival_goal_distance, color=text_color, scale=text_scale)
        y_offset += 12
        img.draw_string(5, y_offset, "Ang: %.1f grados" % rival_goal_angle, color=text_color, scale=text_scale)
        y_offset += 15
        
        # Datos de portería propia (AZUL)
        img.draw_string(5, y_offset, "PORTERIA PROPIA:", color=(0, 0, 255), scale=text_scale)
        y_offset += 12
        img.draw_string(5, y_offset, "Dist: %.1f px" % own_goal_distance, color=text_color, scale=text_scale)
        y_offset += 12
        img.draw_string(5, y_offset, "Ang: %.1f grados" % own_goal_angle, color=text_color, scale=text_scale)
        
        # ENVIAR DATOS POR UART 
        # Formato: "dist_pelota ang_pelota dist_rival ang_rival dist_propia ang_propia\n"
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
        
        # Delay entre iteraciones
        pyb.delay(LOOP_DELAY)

# EJECUCIÓN 

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n=== PROGRAMA DETENIDO POR USUARIO ===")
    except Exception as e:
        print("\n=== ERROR ===")
        print(e)