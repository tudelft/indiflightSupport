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
COPY requirements.txt /requirements.txt
RUN pip install -r /requirements.txt

# copy sim code
COPY PyNDIflight /PyNDIflight

EXPOSE 5000

WORKDIR /
ENTRYPOINT [ "python3", "sim.py" ]
