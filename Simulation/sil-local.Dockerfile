FROM indiflight

RUN apt-get update \
    && apt-get --no-install-recommends install -y \
        gdb \
        gdbserver \
    && apt-get clean

# do requirements first, so rebuilding caches this even if Simulation code changes
COPY Simulation/requirements.txt /requirements.txt
RUN pip install -r /requirements.txt

COPY LogAnalysis/indiflightLogTools/requirements.txt /requirements2.txt
RUN pip install -r /requirements2.txt

# copy sim code
COPY Simulation/PyNDIflight /PyNDIflight

# copy data analysis code
COPY LogAnalysis/indiflightLogTools /indiflightLogTools

EXPOSE 5000
EXPOSE 3333

ENV EXTRA="-Wno-double-promotion -Wno-misleading-indentation"

RUN echo "#!/bin/bash" > /entrypoint.sh &&                      \
    echo "set -e" >> /entrypoint.sh &&                          \
    echo "cd /indiflight" >> /entrypoint.sh &&                  \
    echo 'make -j TARGET=MOCKUP DEBUG=GDB EXTRA_FLAGS_CMDLINE="$EXTRA"' >> /entrypoint.sh && \
    echo "cd /" >> /entrypoint.sh &&                            \
    echo 'gdbserver localhost:3333 /usr/bin/python3 /sim.py "$@";' >> /entrypoint.sh &&                      \
    chmod +x /entrypoint.sh

WORKDIR /
ENTRYPOINT [ "/entrypoint.sh", \
    "--sil", "/indiflight/obj/main/indiflight_MOCKUP.so", \
    "--sil-profile-txt", "/profile.txt"]
