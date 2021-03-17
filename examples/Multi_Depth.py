import cv2
import numpy as np
from PIL import Image
from time import sleep

import pyk4a
from helpers import colorize
from pyk4a import Config, PyK4A


def determine_cams(camera1, camera2):
    camera1.start()
    #sleep(50/1000)
    camera2.start()
    # Check for master/client relation for sync
    if camera1.sync_jack_status[1] == True and camera2.sync_jack_status[0] == True:
        camera1.stop()
        camera2.stop()
        return camera1, camera2
    elif camera1.sync_jack_status[0] == True and camera2.sync_jack_status[1] == True:
        camera1.stop()
        camera2.stop()
        return camera2, camera1
    else:
        camera1.stop()
        camera2.stop()
        print("Correct sync configuration not detected")
        sys.exit()
        return None, None


def main():
    k4a = PyK4A(
    Config(
        color_resolution=pyk4a.ColorResolution.OFF,
        depth_mode=pyk4a.DepthMode.WFOV_2X2BINNED,
        synchronized_images_only=False,
        camera_fps=pyk4a.FPS.FPS_15,
    )
    )
    k4a2 = PyK4A(
    Config(
        color_resolution=pyk4a.ColorResolution.OFF,
        depth_mode=pyk4a.DepthMode.WFOV_2X2BINNED,
        synchronized_images_only=False,
        camera_fps=pyk4a.FPS.FPS_15,
    )
    )
    k4a2._device_id = 1

    Left, Right = determine_cams(k4a, k4a2)
    Left._config.color_resolution = pyk4a.ColorResolution.RES_720P
    # Including color for the second camera stops the system from working, likely a USB bandwidth issue
    # Right._config.color_resolution = pyk4a.ColorResolution.RES_720P
    Left._config.wired_sync_mode = pyk4a.WiredSyncMode.MASTER
    Right._config.wired_sync_mode = pyk4a.WiredSyncMode.SUBORDINATE
    #Left._config.subordinate_delay_off_master_usec = 160
    Right._config.subordinate_delay_off_master_usec = 16

    # This may only work for multiple RBG cameras. Not sure
    #Left._config.synchronized_images_only = True
    #Right._config.synchronized_images_only = True

    
    #sleep(50/1000)
    Right.start()

    Left.start()



    # getters and setters directly get and set on device
    Left.whitebalance = 4500
    assert Left.whitebalance == 4500
    Left.whitebalance = 4510
    assert Left.whitebalance == 4510
    Right.whitebalance = 4500
    assert Right.whitebalance == 4500
    Right.whitebalance = 4510
    assert Right.whitebalance == 4510
    i = 0
    while True:
        #sleep(50/1000)
        capture = Left.get_capture()
        capture2 = Right.get_capture()
        if np.any(capture.depth) and np.any(capture2.depth):
            LeftImage = colorize(capture.depth, (None, 5000), cv2.COLORMAP_HSV)
            RightImage = colorize(capture2.depth, (None, 5000), cv2.COLORMAP_HSV)
            if i ==5 :
                Left.save_calibration_json('2ndDepthleft.json')
                Right.save_calibration_json('2ndDepthright.json')

                
            cv2.imshow("Left", LeftImage)
            cv2.imshow("Right", RightImage)
            Image.fromarray(LeftImage).save(f"ImageCapture/{i}Left.jpeg")
            Image.fromarray(RightImage).save(f"ImageCapture/{i}Right.jpeg")
            np.savetxt(f"ImageCapture/{i}Left.csv", np.asarray(capture.depth), delimiter=',')
            np.savetxt(f"ImageCapture/{i}Right.csv", np.asarray(capture2.depth), delimiter=',')
            print(i)
            i+=1
            key = cv2.waitKey(10)
            if key != -1:
                cv2.destroyAllWindows()
                print("key != -1 for capture")
                break

    Right.stop()
    Left.stop()


if __name__ == "__main__":
    print("Example of two cameras at once")
    main()
