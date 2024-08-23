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

## Steps

Build container (this also compiles the correct version of `indiflight`)

    docker build . -t pyndiflight-iros2024 -f Simulation/sil-githash.Dockerfile \
        --build-arg COMMIT=ab5c74c66                                            \
        --build-arg BUILD_CONFIG=LogAnalysis/IROS2024/IndiflightConfig.mk

Run container (`--num 1000` was used in the paper, but takes long to sim and to analyze)

    docker run -it -p 5000:5000                                       \
        -v ./LogAnalysis/IROS2024/simRandomQuad.py:/sim.py            \
        -v ./LogAnalysis/IROS2024/IndiflightProfile.txt:/profile.txt  \
        pyndiflight-iros2024 --sil-log --no-real-time --num 20

Copy logs

    docker cp $(docker ps -alq):/logs/. ./LogAnalysis/IROS2024/IROS2024_SimulationData

Generate plots and tables (they will appear in `LogAnalysis/IROS2024/figures`):

    pip install -r LogAnalysis/indiflightLogTools/requirements.txt
    python3 LogAnalysis/IROS2024/generateFigures.py
    python3 LogAnalysis/IROS2024/generateVideo.py

*NOTE*: the original plots were generated with `"text.usetex": True`, but that requires
a Latex installation. For portability, this is turned off by default here.
