from argparse import ArgumentParser
from .code import process_pose
from .data import PoseWithCovarianceStamped

def parse_args() -> ArgumentParser:
    parser = ArgumentParser(description="nrai-perception")
    return parser

def main() -> None:
    args = parse_args().parse_args()
    print("nrai-perception main executed with args:", args)
    out = process_pose(PoseWithCovarianceStamped())
    print("Processed pose:", out)

if __name__ == "__main__":
    main()
