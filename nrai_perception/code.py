import cv2
from ultralytics import YOLO

model = YOLO("path to model.py") #path of you best model

def process_pose(msg: object) -> object:
    # Do some processing on the pose message
    # ... processing logic ...
    return msg


def process_camera(msg: object) -> object:
    print(f"{msg.width}x{msg.height}")


def process_image(msg: object) -> object:
    print(msg.height, msg.width, msg.data.shape, len(msg.data) / (msg.height * msg.width))
    img = msg.data.reshape((msg.height, msg.width, -1))[:, :, :3] # Remove the alpha channel from the RGBA

    results = model.predict(img, conf=0.5) #path to test image

    # Process results list
    for i, result in enumerate(results):
        boxes = result.boxes  # Boxes object for bounding box outputs
        print(f"Number of results: {len(result.boxes.cls)}")

        print(f"Boxes for result {i+1}:")
        print(boxes)

        # Get the annotated image as a NumPy array
        im_array = result.plot(labels=False, conf=False, show=True)  # show=False to prevent immediate display

    cv2.imshow("image", img)
    cv2.waitKey()
