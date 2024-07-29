FROM ubuntu:22.04

RUN apt-get update && \
    apt-get --no-install-recommends install -y \
        build-essential \ 
        gcc-11 \
        python3 \
        python3-pip \
        python3-dev \
        git \
    && apt-get clean

RUN pip install --upgrade pip

ADD external/indiflight /indiflight
WORKDIR /indiflight
RUN make TARGET=MOCKUP

ADD Simulation/Python /sim
WORKDIR /sim
RUN pip install -r requirements.txt

EXPOSE 5000

ENTRYPOINT [ "python3", "/sim/sim.py", \
    "--sil-mockup", "/indiflight/obj/main/indiflight_MOCKUP.so", \
    "--sil-profile-txt", "/sim/configs/BTFL_cli_20240314_MATEKH743_CineRat_HoverAtt_NN.txt"]
