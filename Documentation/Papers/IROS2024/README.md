# Control of Unknown Quadrotors from a Single Throw

To reproduce the simulation results (and figure/videos from experimental data) 
of our IROS 2024 paper "Control of Unknown Quadrotors from a Single Throw", 
install docker (https://docs.docker.com/engine/install/) and follow the steps
below. 

**NOTE**: Only tested on Ubuntu 22.04.

**NOTE 2**: All commands have to be run from the root of this repository, not from this 
directory.

**NOTE 3**: To modify the randomization parameters, see `simRandomQuad.py`,

**NOTE 4**: To fly custom configurations in simulation, see `Simulation/README.md` folder.

## Steps to reproduce Simulation

Build container (this also compiles the correct version of `indiflight`)

    docker build . -t pyndiflight-iros2024 -f Simulation/sil-githash.Dockerfile \
        --build-arg COMMIT=85e336c1e                                            \
        --build-arg BUILD_CONFIG=Documentation/Papers/IROS2024/IndiflightConfig.mk

Run container (`--num 1000` was used in the paper, but takes long to sim and to analyze)

    docker run -it -p 5000:5000                                                \
        -v ./Documentation/Papers/IROS2024/simRandomQuad.py:/sim.py            \
        -v ./Documentation/Papers/IROS2024/IndiflightProfile.txt:/profile.txt  \
        pyndiflight-iros2024 --sil-log --no-real-time --num 20

Copy logs

    docker cp $(docker ps -alq):/logs/. ./logs-simulation


## Steps to reproduce plots from dataset or simulation output

Create a new python venv, just to be sure about dependencies

    python -m venv ~/.indiflight-venv
    source ~/.indiflight-venv/bin/activate

Install dependencies:

    pip install --upgrade pip
    pip install -r LogAnalysis/indiflightLogTools/requirements.txt
    pip install -r Simulation/requirements.txt

Download the dataset <LINK> on some location on your machine. If you want, substitute the simulation data with the output of your own simulations above

Generate plots and tables (they will appear in `LogAnalysis/IROS2024/figures`):

    python3 Documentation/Papers/IROS2024/generateFigures.py /path/to/dataset
    python3 Documentation/Papers/IROS2024/generateVideo.py /path/to/dataset

*NOTE*: unfortunately, importing the binary datafiles is quite slow...
*NOTE*: the original plots were generated with `"text.usetex": True`, but that requires
a Latex installation. For portability, this is turned off by default here.
*NOTE*: running this will create cache files of the data, so they can be imported faster in the future. To delete the cache, run `python3 -c "from LogAnalysis.indiflightLogTools import IndiflightLog; IndiflightLog.clearCache()"`
