import cv2
import numpy as np
from PIL import Image
from time import sleep

import pyk4a
from helpers import colorize
from pyk4a import Config, PyK4A

data = np.genfromtxt("ImageCapture/4Right.csv", delimiter=',')
RightImage = colorize(data, (None, 5000), cv2.COLORMAP_HSV)

def refresh():
    cv2.imshow("Left", RightImage)
    key = cv2.waitKey(10)
