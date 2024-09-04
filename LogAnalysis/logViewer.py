from dash import Dash, dcc, html, Input, Output
import plotly.graph_objs as go
from plotly_resampler import FigureResampler
import numpy as np

from indiflightLogTools import IndiflightLog
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
import sys
from os import path
import logging

import signal
import ipdb
def signal_handler(sig, frame):
    print("##### Enter 'c' to continue, then manually reload webpage. Press 'CTRL+D' to exit")
    ipdb.set_trace()
    signal.signal(signal.SIGINT, signal_handler)


app = Dash(__name__)
VERBOSITIES = [logging.ERROR, logging.WARNING, logging.INFO, logging.DEBUG]

layout = dict(
    hoversubplots="axis",
    title="Stock Price Changes",
    hovermode="x unified",
    grid=dict(rows=2, columns=1),
)


app.layout = html.Div([ dcc.Tabs([]) ])
tabs = app.layout.children[0].children

if __name__=="__main__":
    parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument("datafile", help="single indiflight bfl logfile")
    parser.add_argument("--range","-r", required=False, nargs=2, type=int, help="integer time range to consider in ms since start of datafile")
    parser.add_argument("-c", "--compare", required=False, help="supply a second file for comparison")
    parser.add_argument("-o", "--offset", required=False, type=int, help="ms to shift the second log forward in time")
    parser.add_argument("-n", "--names", required=False, nargs=2, type=str, default=("A", "B"), help="2 names to use for the files in legend")
    parser.add_argument("-v", required=False, action='count', default=0, help="verbosity (can be given up to 3 times)")
    parser.add_argument("--no-cache", required=False, action='store_true', default=False, help="Do not load from or store to raw data cache")
    parser.add_argument("--clear-cache", required=False, action='store_true', default=False, help="Clear raw data cache")
    parser.add_argument("--use-reloader", required=False, action='store_true', default=False, help="Reload page on saving this script")

    # clear cache, even if no other arguments are given
    if "--clear-cache" in sys.argv[1:]:
        IndiflightLog.clearCache()
        if len(sys.argv) == 2:
            exit(0)

    args = parser.parse_args()

    logging.basicConfig(
        format='%(asctime)s -- %(name)s %(levelname)s: %(message)s',
        level=VERBOSITIES[min(args.v, 3)],
        )

    if args.compare:
        raise NotImplementedError("Second logfile not implemented yet")

    if not args.use_reloader:
        signal.signal(signal.SIGINT, signal_handler)

    log = IndiflightLog(args.datafile, args.range, not args.no_cache)
    time_ms = log.data['timeMs'].to_numpy()

    #%% Tab 1
    newTab = dcc.Tab(label="Number One", children=[])

    theLayout = layout.copy()
    theLayout['title'] = 'Gyro'
    theLayout['grid']['rows'] = 3

    fig = FigureResampler(go.Figure(layout=theLayout))
    fig.register_update_graph_callback(app, "shared-x-graph")
    for i in range(3):
        fig.add_trace(go.Scattergl(mode='lines', name="Gyro", xaxis="x", yaxis=f"y1"),
            hf_x=time_ms, hf_y=log.data[f"gyroADCafterRpm[{i}]"])
    for i in range(3):
        fig.add_trace(go.Scattergl(mode='lines', name="Acc", xaxis="x", yaxis=f"y2"),
            hf_x=time_ms, hf_y=log.data[f"accADCafterRpm[{i}]"])

    newTab.children.append(
        dcc.Graph(id='shared-x-graph', figure=fig, style={"height": 800})  # Pass the Plotly figure directly to the Dash Graph component
    )
    tabs.append(newTab)

    #%%
    app.run_server(debug=True, use_reloader=args.use_reloader)

