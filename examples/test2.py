import cv2
import numpy as np

import pyk4a
from helpers import colorize
from pyk4a import Config, PyK4A


def main():
    k4a = PyK4A(
        Config(
            color_resolution=pyk4a.ColorResolution.OFF,
            depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
            synchronized_images_only=False,
        )
    )
    k4a2 = PyK4A(
        Config(
            color_resolution=pyk4a.ColorResolution.OFF,
            depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
            synchronized_images_only=False,
        )
    )
    k4a2._device_id = 1
    k4a.start()
    k4a2.start()

    # getters and setters directly get and set on device
    k4a.whitebalance = 4500
    assert k4a.whitebalance == 4500
    k4a.whitebalance = 4510
    assert k4a.whitebalance == 4510

    k4a2.whitebalance = 4500
    assert k4a2.whitebalance == 4500
    k4a2.whitebalance = 4510
    assert k4a2.whitebalance == 4510

    while True:
        capture = k4a.get_capture()
        capture2 = k4a2.get_capture()
        if np.any(capture.depth):
            cv2.imshow("k4a", colorize(capture.depth, (None, 5000), cv2.COLORMAP_HSV))
            key = cv2.waitKey(10)
            if key != -1:
                cv2.destroyAllWindows()
                break
        if np.any(capture2.depth):
            cv2.imshow("k4a2", colorize(capture2.depth, (None, 5000), cv2.COLORMAP_HSV))
            key2 = cv2.waitKey(10)
            if key2 != -1:
                cv2.destroyAllWindows()
                break
    k4a.stop()
    k4a2.stop()


if __name__ == "__main__":
    print("Example of two cameras at once")
    main()
