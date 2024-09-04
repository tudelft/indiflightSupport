import numpy as np
from indiflightLogTools import IndiflightLog
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
from matplotlib import pyplot as plt

if __name__=="__main__":
    # script to demonstrate how to use it
    parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument("datafile", help="single indiflight bfl logfile")
    parser.add_argument("--range","-r", required=False, nargs=2, type=int, help="integer time range to consider in ms since start of datafile")
    args = parser.parse_args()

    log = IndiflightLog(args.datafile, args.range)
