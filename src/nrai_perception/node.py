import os
from .code import ZEDYOLOTrack
import pickle
import argparse
from multiprocessing import Queue
import numpy as np
import logging

logger = logging.getLogger()

fifo_path = '/tmp/PERCEPTION_ZedYoloTrack'

def handle_zed(args: argparse.Namespace):
    try:
        import pyzed.sl as sl
    except ImportError:
        raise ImportError("NRAI_PERCEPTION: ZED python API not installed, ZED SDK likely not installed.")

    # --- Set up ZED Camera ---
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.sdk_verbose=0
    
    if zed.open(init_params) > sl.ERROR_CODE.SUCCESS:
        logger.error("NRAI_PERCEPTION: Unable to open ZED camera, aborting.")
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

def handle_simulator(args: argparse.Namespace):
    topics: dict[str, Queue] = args.topics or {}

    # --- Set up Code ---
    if args.camera_topic not in topics:
        raise ValueError(f"No '{args.camera_topic}' topic to listen to.")

    camera_queue = topics[args.camera_topic]
    planning_queue = topics.get(args.planning_topic, None)
    node = ZEDYOLOTrack()
    while True:
        while camera_queue.qsize()>1:
            camera_queue.get()
        image: np.ndarray = camera_queue.get()
        logger.debug("Received %s", image.shape)


        if image.dtype == np.uint8:
            new_instruction = node.rgb_callback(image)
        else:
            node.depth_callback(image)
            continue
        
        if planning_queue is not None and new_instruction is not None:
            planning_queue.put(new_instruction)
            logger.debug("Sent %s", new_instruction)

def main(args: argparse.Namespace | None = None):
    args = args or argparse.Namespace()
    logging.basicConfig(format=args.logger_format or "", level=args.actual_verbosity() if args.actual_verbosity else logging.INFO)
    logger.info("Initializing...")
    try:
        if args.zed:
            handle_zed(args)
        else:
            handle_simulator(args)
    except KeyboardInterrupt:
        logger.info("Closing...")

if __name__ == "__main__":
    main()