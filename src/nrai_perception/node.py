import os
from .code import ZEDYOLOTrack
import pickle
import argparse
from multiprocessing import Queue
import numpy as np
import logging

logger = logging.getLogger()

lap_count = 0
lap_state = False # Where True is about to lap, and False is pre-lap / post-lap.

def connect_socket():
    while True:
        try:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            s.connect(socket_path)
            logger.info("Connected to NIMS")
            return s
        except:
            logger.error("Could not connect to NIMS, retrying in 1 second...")
            sleep(1)

def send_lap(sock):
    packet = struct.pack("<BI", 0x06, 0x00000000)
    try:
        sock.sendall(packet)
        return True
    except:
        return False

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
    
    # --- Execute loop ---
    image, depth = sl.Mat(), sl.Mat()
    runtime_parameters = sl.RuntimeParameters()

    camera_queue = topics[args.camera_topic]
    planning_queue = topics.get(args.planning_topic, None)

    while True:
        if zed.grab(runtime_parameters):
            while camera_queue.qsize() > 1:
                logger.debug("Emptying queue")
                camera_queue.get()

            try:
                zed.retrieve_image(image, sl.VIEW.LEFT)
                zed.retrieve_measure(depth, sl.MEASURE.DEPTH)

                node.depth_callback(depth.get_data())
                new_instruction, orange_cones = node.rgb_callback(image.get_data())
                if new_instruction is None: raise Exception("Depth not yet initialised")
            except:
                continue
            # Pass forward cone positions
            if planning_queue is not None and new_instruction is not None:
                planning_queue.put(new_instruction)
                logger.debug("Sent %s", new_instruction)

def handle_simulator(args: argparse.Namespace):
    global lap_count
    global lap_state
    topics: dict[str, Queue] = args.topics or {}

    # --- Set up Code ---
    if args.camera_topic not in topics:
        raise ValueError(f"No '{args.camera_topic}' topic to listen to.")

    camera_queue = topics[args.camera_topic]
    planning_queue = topics.get(args.planning_topic, None)
    node = ZEDYOLOTrack()
    while True:
        logger.debug("Starting loop")
        while camera_queue.qsize() > 1:
            logger.debug("Emptying queue")
            camera_queue.get()
        image: np.ndarray = camera_queue.get()
        logger.debug("Received %s", image.shape)

        if image.dtype == np.uint8:
            new_instruction, orange_cones = node.rgb_callback(image)
            if new_instruction==None: continue

            # State machine for lap detection
            if not lap_state:
                # No immenent lap detected
                if not orange_cones:
                    lap_count=0
                else:
                    lap_count+=1
                if lap_count>2:
                    lap_state=True
            elif not orange_cones:
                # Immenent lap detected and no orange cones
                lap_count -= 1
                if lap_count<=0:
                    # Lap detected.
                    lap_state = False
                    while not send_lap(s):
                        # If this loops forever, NIMS is dead anyways.
                        s = connect_socket()
            else:
                # Immenent lap detected and orange cones
                lap_count = 3
                
        else:
            node.depth_callback(image)
            continue
        
        if planning_queue is not None and new_instruction is not None:
            planning_queue.put(new_instruction)
            logger.debug("Sent %s", new_instruction)

def main(args: argparse.Namespace):
    logging.basicConfig(format=args.logger_format or "", level=args.actual_verbosity() if args.actual_verbosity else logging.INFO)
    logger.info("Initializing...")
    if args.zed:
        handle_zed(args)
    else:
        handle_simulator(args)

if __name__ == "__main__":
    main()