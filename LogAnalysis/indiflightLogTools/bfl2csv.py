#!/usr/bin/env python3
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
from indiflight_log_importer import IndiflightLog
from os import path

if __name__=="__main__":
    parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument("logfile", metavar="x.BFL", help="BFL log to be converted")
    parser.add_argument("-o", "--output-dir", required=False, type=str, default=None, help="Output Directory. If no passed, same directory as logfile.")

    args = parser.parse_args()
    dir = args.output_dir if args.output_dir else path.dirname(args.logfile)
    stem = path.splitext(path.basename(args.logfile))[0]
    output = path.join(dir, f"{stem}.csv")

    log = IndiflightLog(args.logfile)
    log.data.to_csv(output)
