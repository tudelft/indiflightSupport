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
COPY Simulation/requirements.txt /requirements.txt
RUN pip install -r /requirements.txt

# copy sim code
COPY Simulation/PyNDIflight /PyNDIflight

# download and build INDIflight mockup
# adding the heads is just to ensure that git clone runs when there is new commits
#ADD https://api.github.com/repos/tudelft/indiflight/git/refs/heads version.json
ARG COMMIT
RUN git clone https://github.com/tudelft/indiflight.git /indiflight
WORKDIR /indiflight
RUN git reset --hard ${COMMIT}
RUN git submodule update --init
RUN pip install -r /indiflight/lib/main/pi-protocol/python/requirements.txt

ARG BUILD_CONFIG
COPY ${BUILD_CONFIG} /indiflight/make/local.mk
RUN make TARGET=MOCKUP DEBUG=GDB

EXPOSE 5000

WORKDIR /
ENTRYPOINT [ "python3", "/sim.py", \
    "--sil", "/indiflight/obj/main/indiflight_MOCKUP.so", \
    "--sil-profile-txt", "/profile.txt"]
