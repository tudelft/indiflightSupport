#!/usr/bin/env python3
from argparse import ArgumentParser, ArgumentError
from matplotlib import pyplot as plt
import logging
import sys

from indiflightLogTools import IndiflightLog

if __name__=="__main__":
    # script to demonstrate how to use it
    parser = ArgumentParser()
    parser.add_argument("datafile", help="single indiflight bfl logfile")
    parser.add_argument("--range","-r", required=False, nargs=2, type=int, help="integer time range to consider in ms since start of datafile")
    parser.add_argument("-c", "--compare", required=False, help="supply a second file for comparison")
    parser.add_argument("-o", "--offset", required=False, type=int, help="ms to shift the second log forward in time")
    parser.add_argument("-n", "--names", required=False, nargs=2, type=str, default=("A", "B"), help="2 names to use for the files in legend")
    parser.add_argument("-v", required=False, action='count', default=0, help="verbosity (can be given up to 3 times)")
    parser.add_argument("--no-cache", required=False, action='store_true', default=False, help="Do not load from or store to raw data cache")
    parser.add_argument("--clear-cache", required=False, action='store_true', default=False, help="Clear raw data cache")

    # clear cache, even if no other arguments are given
    if "--clear-cache" in sys.argv[1:]:
        IndiflightLog.clearCache()
        if len(sys.argv) == 2:
            exit(0)

    args = parser.parse_args()

    verbosity = [logging.ERROR, logging.WARNING, logging.INFO, logging.DEBUG]
    logging.basicConfig(
        format='%(asctime)s -- %(name)s %(levelname)s: %(message)s',
        level=verbosity[min(args.v, 3)],
        )

    # import data
    log = IndiflightLog(args.datafile, args.range, not args.no_cache)

    from logTools.estimators import imuOffsetCorrection, Signal
    import numpy as np

    a = log.data[[f"accUnfiltered[{i}]" for i in range(3)]].to_numpy()
    w = log.data[[f"gyroADCafterRpm[{i}]" for i in range(3)]].to_numpy()
    aSig = Signal(log.data["timeS"], a).filter('lowpass', 2, 25)
    wSig = Signal(log.data["timeS"], w).filter('lowpass', 2, 25)
    dw = wSig.dot();
    r = np.array([-0.010, -0.010, 0.015])
    #r = np.array([-0.01, -0.005, 0.000])
    #r = np.array([-0.010, -0.015, 0.010])
    f_cor = imuOffsetCorrection(aSig.y.copy(), wSig.y.copy(), dw.y.copy(), r)

    log.data[[f"accSmooth[{i}]" for i in range(3)]] = f_cor

    # plot some stuff
    if args.compare:
        i, f = args.range
        i += args.offset
        f += args.offset
        second = IndiflightLog(args.compare, (i, f), not args.no_cache)
        f = log.compare(second,
                        other_offset=args.offset,
                        self_name=args.names[0],
                        other_name=args.names[1])
        plt.show()
    else:
        f = log.plot()
        plt.show()

    # example:
    # ./logPlotter.py -vv LOG00004_CineRat_CatapultDelayLearningSequence.BFL -r 1300 1933
    # ./logPlotter.py -v LOG00004_CineRat_CatapultDelayLearningSequence.BFL -r 1300 1933 --compare LOG00003_CatapultDelayLearningSequence.BFL --offset -2 --names "real" "sim"

    # in case the pickle cache dir (run with -vv to find which one is used) becomes too large:
    # ./logPlotter.py --clear-cache
