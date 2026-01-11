#pelota roja con base a la forma y el color


import sensor, image, time
from pyb import UART, LED

# --- CONFIGURACIÓN DE CÁMARA ---
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)


# --- LEDs INDICADORES ---
led_rojo = LED(1)    # LED rojo
led_verde = LED(2)   # LED verde
led_azul = LED(3)    # LED azul

# --- UMBRAL DE COLOR ROJO EN LAB ---
# Formato: (L Min, L Max, A Min, A Max, B Min, B Max)
# Para rojo: A alto (positivo indica rojo)
red_threshold = (30, 100, 15, 127, 15, 127)
blue_threshold = (0, 35, 33, -10, -122, -14)
yellow_treshold = (50, 73, -55, 48, 8, 28)


# --- PARÁMETROS ---
ANCHO = 320
ALTO = 240


ZONA_MUERTA = 50          # Tolerancia en píxeles
AREA_MINIMA = 100         # Área mínima del blob
RADIO_MINIMO = 20         # Radio mínimo para acercarse
PROPORCION_LINEA = 0.7    # Para detectar líneas horizontales

CENTRO_X = ANCHO // 2
CENTRO_Y = ALTO // 2

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    
    pelota_detectada = False
    linea_detectada = False
    
    # Buscar blobs rojos
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
                

    
    # --- SI NO SE DETECTA NADA ---
    if not pelota_detectada
        img.draw_string(10, 10, "BUSCANDO...", color=(255, 255, 0), scale=2)
        print("Buscando pelota...")
        
        led_rojo.off()
        led_verde.off()
        led_azul.toggle()  # Parpadeo azul
    
    # Mostrar FPS
    print("FPS: %d" % clock.fps())