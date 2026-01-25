import sensor
import time
import utime
import pyb
import math
import image
from pyb import UART

# ================= UART =================
uart = UART(3, 115200, timeout_char=0)
uart.init(115200, bits=8, parity=None, stop=1)

# =============== THRESHOLDS ===============
TH_BALL = (35, 85, 25, 75, 10, 70)
TH_GOAL_Y = (50, 73, -55, 48, 8, 28)
TH_GOAL_B = (35, 100, -128, -10, -128, -10)

# =============== CENTRO ==================
X_CENTER = 195
Y_CENTER = 169

# =============== INIT ====================
def initialize_open():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=3000)

    sensor.set_auto_gain(False)
    sensor.set_gainceiling(16)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=19000)

    sensor.set_brightness(-1)
    sensor.set_contrast(-3)
    sensor.set_saturation(-3)

    sensor.set_hmirror(True)
    sensor.set_transpose(True)

# =============== UTIL ====================
def get_biggest_blob(blobs):
    return max(blobs, key=lambda b: b.area()) if blobs else None

def distance(cx, cy):
    return math.sqrt((cx - X_CENTER)**2 + (cy - Y_CENTER)**2)

def angle(cx, cy):
    a = math.degrees(math.atan2(cy - Y_CENTER, cx - X_CENTER))
    return a + 360 if a < 0 else a

# =============== MAIN ====================
def main():
    initialize_open()
    clock = time.clock()

    while True:
        clock.tick()
        img = sensor.snapshot()

        # ===== PELOTA =====
        distance_b = angle_b = 0
        blobs = img.find_blobs([red_threshold], 
                          pixels_threshold=200, 
                          area_threshold=AREA_MINIMA,
                          merge=True)
    
    if blobs:
        # Ordenar por área (mayor primero)
        blobs = sorted(blobs, key=lambda b: b.area(), reverse=True)
        
        for blob in blobs:
            # Calcular aspect ratio
            aspect_ratio = blob.w() / float(blob.h()) if blob.h() > 0 else 0
            
            
            # --- DETECCIÓN DE PELOTA ---
            # Verificar circularidad (compacidad)
            circularidad = blob.compactness()
            
            if circularidad > 0.5:  # Objeto circular
                pelota_detectada = True
                
                # Dibujar detección
                img.draw_rectangle(blob.rect(), color=(0, 255, 0))
                img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
                
                # Calcular desplazamiento
                desp_x = blob.cx() - CENTRO_X
                desp_y = blob.cy() - CENTRO_Y
                
                # Estimar distancia por el radio
                radio = (blob.w() + blob.h()) / 4
                
                
                # Mostrar info en pantalla
                img.draw_string(10, 10, "R:%d X:%d" % (int(radio), desp_x), 
                              color=(255, 255, 255), scale=1)

        # ===== PORTERÍA AMARILLA =====
        distance_g = angle_g = 0
        blobs = img.find_blobs([TH_GOAL_Y], pixels_threshold=200, area_threshold=800, merge=True)
        blob = get_biggest_blob(blobs)

        if blob:
            img.draw_rectangle(blob.rect(), color=(0,255,0))
            img.draw_cross(blob.cx(), blob.cy(), color=(0,255,0))
            distance_g = distance(blob.cx(), blob.cy())
            angle_g = -(angle(blob.cx(), blob.cy()) - 180)

        # ===== PORTERÍA AZUL =====
        distance_gop = angle_gop = 0
        blobs = img.find_blobs([TH_GOAL_B], pixels_threshold=200, area_threshold=800, merge=True)
        blob = get_biggest_blob(blobs)

        if blob:
            img.draw_rectangle(blob.rect(), color=(0,0,255))
            img.draw_cross(blob.cx(), blob.cy(), color=(0,0,255))
            distance_gop = distance(blob.cx(), blob.cy())
            angle_gop = -(angle(blob.cx(), blob.cy()) - 180)

        # ===== UART =====
        data = "{} {} {} {} {} {}\n".format(distance_b, angle_b, distance_g, angle_g, distance_gop, angle_gop)
        print("Sending: ", data)
        uart.write(data)
        pyb.delay(50)



# =============== START ====================
main()
