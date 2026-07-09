from .node import main as _main
import argparse


def main(args: argparse.Namespace):
    try:
        _main(args)
    except KeyboardInterrupt:
        pass
