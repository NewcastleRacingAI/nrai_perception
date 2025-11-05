import cv2
import numpy as np


def process_pose(msg: object) -> object:
    # Do some processing on the pose message
    # ... processing logic ...
    return msg


def process_camera(msg: object) -> object:
    print(f"{msg.width}x{msg.height}")


def process_image(msg: object) -> object:
    print(msg.height, msg.width, msg.data.shape, len(msg.data) / (msg.height * msg.width))
    img = msg.data.reshape((msg.height, msg.width, -1))

    cv2.imwrite("color_img.jpg", img)
    cv2.imshow("image", img)
    cv2.waitKey()
