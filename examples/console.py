import cv2
import sys
import numpy as np
import pyk4a
from helpers import colorize
from pyk4a import Config, PyK4A


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


def determine_cams(camera1, camera2):
    camera1.start()
    camera2.start()
    # Check for master/client relation for sync
    if camera1.sync_jack_status[1] == True and camera2.sync_jack_status[0] == True:
        camera1.stop()
        camera2.stop()
        camera2
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
        
Left, Right = determine_cams(k4a, k4a2)
Left.wired_sync_mode=pyk4a.WiredSyncMode.MASTER
Left._config.wired_sync_mode =pyk4a.WiredSyncMode.MASTER
Right.wired_sync_mode=pyk4a.WiredSyncMode.SUBORDINATE
Right._config.wired_sync_mode =pyk4a.WiredSyncMode.SUBORDINATE
Left.subordinate_delay_off_master_usec=160
Right.subordinate_delay_off_master_usec=160

def cams_start():
    Left.start()
    Right.start()
    # getters and setters directly get and set on device
    Left.whitebalance = 4500
    assert Left.whitebalance == 4500
    Left.whitebalance = 4510
    assert Left.whitebalance == 4510
    Right.whitebalance = 4500
    assert Right.whitebalance == 4500
    Right.whitebalance = 4510
    assert Right.whitebalance == 4510



def update_capture():
	capture = Left.get_capture()
	capture2 = Right.get_capture()
	if np.any(capture.depth):
	    cv2.imshow("Left", colorize(capture.depth, (None, 5000), cv2.COLORMAP_HSV))
	    key = cv2.waitKey(10)
	    if key != -1:
	        cv2.destroyAllWindows()
	        print("key != -1 for capture")
	if np.any(capture2.depth):
	    cv2.imshow("Right", colorize(capture2.depth, (None, 5000), cv2.COLORMAP_HSV))
	    key2 = cv2.waitKey(10)
	    if key2 != -1:
	        cv2.destroyAllWindows()
	        print("keykey2 != -1 for capture2")


def stop_cams():
	Left.stop()
	Right.stop()


