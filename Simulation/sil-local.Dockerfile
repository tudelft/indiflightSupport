FROM indiflight

RUN apt-get update \
    && apt-get --no-install-recommends install -y \
        gdb \
        gdbserver \
    && apt-get clean

# do requirements first, so rebuilding caches this even if Simulation code changes
COPY requirements.txt /requirements.txt
RUN pip install -r /requirements.txt

# copy sim code
COPY PyNDIflight /PyNDIflight

EXPOSE 5000
EXPOSE 3333

RUN echo "#!/bin/bash" > /entrypoint.sh &&                      \
    echo "set -e" >> /entrypoint.sh &&                          \
    echo "cd /indiflight" >> /entrypoint.sh &&                  \
    echo "make -j TARGET=MOCKUP DEBUG=GDB" >> /entrypoint.sh && \
    echo "cd /" >> /entrypoint.sh &&                            \
    echo 'gdbserver localhost:3333 /usr/bin/python3 /sim.py "$@";' >> /entrypoint.sh &&                      \
    chmod +x /entrypoint.sh

WORKDIR /
ENTRYPOINT [ "/entrypoint.sh", \
    "--sil", "/indiflight/obj/main/indiflight_MOCKUP.so", \
    "--sil-profile-txt", "/profile.txt"]