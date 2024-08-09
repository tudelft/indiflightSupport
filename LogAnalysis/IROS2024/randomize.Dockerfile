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

# do requirements first, so rebuilding caches this even if Simulation code changes
ADD Simulation/Python/requirements.txt /requirements.txt
RUN pip install -r /requirements.txt

# build INDIflight mockup
ADD external/indiflight /indiflight
WORKDIR /indiflight
RUN make TARGET=MOCKUP

ADD Simulation/Python /sim
WORKDIR /sim

EXPOSE 5000
WORKDIR /
ENTRYPOINT [ "python3", "/sim/quadSILRandomize.py", \
    "/indiflight/obj/main/indiflight_MOCKUP.so", \
    "--sil-profile-txt", "/sim/configs/SIL.txt", \
    "--sil-log"]


# TODO: motor commands seem broken in logs? 
