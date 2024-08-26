#!/usr/bin/env python3

# Convert .BFL binary logs to .csv in SI units
#
# Copyright 2024 Till Blaha (Delft University of Technology)
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <https://www.gnu.org/licenses/>.

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
