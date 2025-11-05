from argparse import ArgumentParser, Namespace
from .code import process_pose, process_camera, process_image
from nrai_rosutils import RosBag

class AppNamespace(Namespace):
    input_bag: str
    topic: str
    play: bool

def parse_args() -> ArgumentParser:
    parser = ArgumentParser(description="nrai-perception")
    parser.add_argument("input_bag", type=str, help="Path to input ROS2 bag file", default="")
    parser.add_argument("-t", "--topic", type=str, help="Topic to read from the bag file", default="")
    parser.add_argument("-p", "--play", action="store_true", help="Play back the bag file messages")
    return parser

def process_msg(topic: str, timestamp: float, msg: object) -> object:
    if topic == "/pose":
        return process_pose(msg)
    if topic == "/zed/zed_node/rgb/color/rect/image/camera_info":
        return process_camera(msg)
    if topic == "/zed/zed_node/rgb/color/rect/image":
        return process_image(msg)
    return

def main(args: list[str] | None = None) -> int:
    args: AppNamespace = parse_args().parse_args(args)

    with RosBag(args.input_bag) as bag:
        if args.play:
            for topic, timestamp, msg in bag.play():
                process_msg(topic, timestamp, msg)
            return 0
        if args.topic:
            print(f"Reading messages from topic: {args.topic}")
            print(bag.get_messages(args.topic))
        else:
            print("These are the available topics in the bag file:")
            for topic in bag.topics:
                print(f" - {topic}")

    return 0

if __name__ == "__main__":
    exit(main())
