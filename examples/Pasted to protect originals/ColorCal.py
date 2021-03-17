import cv2
import numpy as np
from PIL import Image
from time import sleep

import pyk4a
from helpers import colorize
from pyk4a import Config, PyK4A


def determine_cams(camera1, camera2):
    camera1.start()
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


def take_image(cam, which, num):
    cam.start()
    # getters and setters directly get and set on device
    cam.whitebalance = 4500
    assert cam.whitebalance == 4500
    while True:
        capture = cam.get_capture()
        if np.any(capture.color):
            img = capture.color
            cv2.imshow(which, img)
            key = cv2.waitKey(10)
            cv2.imwrite(f"ImageCapture/Cali{num}{which}.jpeg", img)
            #img.save(f"ImageCapture/Cali{num}{which}.jpeg")
            #Image.fromarray(RightImage).save(f"ImageCapture/Cali{i}Right.jpeg")

            #cv2.destroyAllWindows()
            #print("key != -1 for capture")
            break
    cam.stop()


def main():
    k4a = PyK4A(
    Config(
        color_resolution=pyk4a.ColorResolution.OFF,
        depth_mode=pyk4a.DepthMode.WFOV_2X2BINNED,
        #color_resolution=pyk4a.ColorResolution.RES_720P,
        #depth_mode=pyk4a.DepthMode.OFF,
        synchronized_images_only=False,
        camera_fps=pyk4a.FPS.FPS_15,
    )
    )
    k4a2 = PyK4A(
    Config(
        color_resolution=pyk4a.ColorResolution.OFF,
        depth_mode=pyk4a.DepthMode.WFOV_2X2BINNED,
        #color_resolution=pyk4a.ColorResolution.RES_720P,
        #depth_mode=pyk4a.DepthMode.OFF,
        synchronized_images_only=False,
        camera_fps=pyk4a.FPS.FPS_15,
    )
    )
    k4a2._device_id = 1
    print("Determining Which camera is where now")
    Left, Right = determine_cams(k4a, k4a2)
    print("\tcamera's have been determined")


    Left._config.color_resolution = pyk4a.ColorResolution.RES_720P
    Right._config.color_resolution = pyk4a.ColorResolution.RES_720P
    Left._config.depth_mode = pyk4a.DepthMode.OFF
    Right._config.depth_mode = pyk4a.DepthMode.OFF

    #Left._config.wired_sync_mode = pyk4a.WiredSyncMode.MASTER
    #Right._config.wired_sync_mode = pyk4a.WiredSyncMode.SUBORDINATE
    #Left._config.wired_sync_modesubordinate_delay_off_master_usec = 160
    #Right._config.wired_sync_modesubordinate_delay_off_master_usec = 160


    i = 0
    while True:
        #sleep(50/1000)
        print("Just in the While Loop")
        take_image(Left, "Left", i)
        take_image(Right, "Right", i)
        delay = input("Press enter to take new image")
        i+=1


if __name__ == "__main__":
    print("Example of two cameras at once")
    main()
