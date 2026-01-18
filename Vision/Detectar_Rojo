import sensor
import time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)\

red_treshold = (30, 100, 15, 127, 15, 127)

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    print(clock.fps())

    blobs = img.find_blobs([red_treshold], pixels_tresholds = 200, area_treshold = 200)

    for blob in blobs:
        img.draw_rectangle(blob.rect(), color = (255, 0, 0))
        img.draw_cross(blob.cx(), blob.cy(), color = (255, 255, 255))
        print("Red objects detected at: ", blob.cx(), blob.cy())