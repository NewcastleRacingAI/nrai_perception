import os
from code import ZEDYOLOTrack
import array
import pickle

try:
    import pyzed.sl as sl
except ImportError:
    raise ImportError("NRAI_PERCEPTION: ZED python API not installed, ZED SDK likely not installed.")

fifo_path = '/tmp/PERCEPTION_ZedYoloTrack'

def main(args=None):
    # --- Set up ZED Camera ---
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.sdk_verbose=0
    
    if zed.open(init_params) > sl.ERROR_CODE.SUCCESS:
        print("NRAI_PERCEPTION: Unable to open ZED camera, aborting.")
        exit(1)
    
    # --- Set up Code ---
    node = ZEDYOLOTrack()
    
    # --- Set up IPC ---
    os.mkfifo(fifo_path, 0o600)
    
    # --- Execute loop ---
    image, depth = sl.Mat(), sl.Mat()
    runtime_parameters = sl.RuntimeParameters()
    while True:
        if zed.grab(runtime_parameters):
            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            
            node.depth_callback(depth.get_data())
            new_instruction = node.rgb_callback(image.get_data())
            
            # Write cone bytes
            try:
                fd = os.open(fifo_path, os.O_WRONLY)
                with open(fd, "wb") as file:
                    pickle.dump(new_instruction, file)
            except FileNotFoundError:
                print(f"NRAI_PERCEPTION: Could not access FIFO {fifo_path}. Likely not yet configured.")
            except BrokenPipeError:
                print(f"NRAI_PERCEPTION: FIFO {fifo_path} terminated")

if __name__ == "__main__":
    main()