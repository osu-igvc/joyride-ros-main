from simple_pyspin import Camera
import cv2

serial = "18295818"

with Camera(serial) as cam:
    cam.init()
    cam.__setattr__('PixelFormat','RGB8Packed')
    cam.__setattr__('VideoMode', 'Mode1')
    print('fps:', cam.__getattr__('AcquisitionFrameRate'))
    cam.start()
        
    while(True):
        img = cv2.cvtColor(cam.get_array(),cv2.COLOR_RGB2BGR)
        cv2.imshow('frame', img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()

## HEY MAX THIS WORKS