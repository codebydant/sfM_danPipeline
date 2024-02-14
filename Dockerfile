FROM hypertools_4_2:latest

RUN apt-get update 

RUN apt-get install -y \
    python3-pip \
    libceres-dev \
    libpcl-dev \
    libvtk7-dev \
    # gdb \
    # gdbserver

