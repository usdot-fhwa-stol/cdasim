FROM ubuntu:18.04

MAINTAINER ODU Team
LABEL Description="Dockerised Simulation of Carla_Sumo_Mosaic"

ENV SUMO_HOME /usr/share/sumo
ENV SUMO_USER carla_sumo_mosaic


ENV CARLA_HOME /CARLA_0.9.10/
RUN export CARLA_HOME

RUN apt-get update && \
    apt-get install -y openjdk-8-jdk && \
    apt-get install -y ant && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/ && \
    rm -rf /var/cache/oracle-jdk8-installer;

ENV JAVA_HOME /usr/lib/jvm/java-8-openjdk-amd64/
RUN export JAVA_HOME

RUN apt-get -y update

RUN apt-get install x11-xserver-utils dconf-editor dbus-x11 -y

RUN apt-get update && apt-get upgrade -y && apt-get clean

# Python package management and basic dependencies
RUN apt-get install -y curl python3.7 python3.7-dev python3.7-distutils

# Register the version in alternatives
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.7 1

# Set python 3 as the default python
RUN update-alternatives --set python /usr/bin/python3.7

RUN curl -s https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python get-pip.py --force-reinstall && \
    rm get-pip.py

RUN pip3.7 install lxml

# Dependencies for glvnd and X11.
RUN apt-get update \
  && apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
  && rm -rf /var/lib/apt/lists/*

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

RUN apt-get update && \
    apt-get install -y software-properties-common && \
    rm -rf /var/lib/apt/lists/*

# Install python3.6 for NS-3 installation
RUN add-apt-repository ppa:deadsnakes/ppa  && \
    apt update && \
    apt install python3.6

# Install other dependencies
RUN apt-get update && \
    apt-get install -y g++ gcc unzip tar python3-dev && \
    apt-get install -y autoconf automake libtool curl make && \
    apt-get install -y libxml2 libsqlite3-dev && \
    apt-get install -y libxml2-dev

RUN \
  apt-get update && \
  apt-get install -y --allow-unauthenticated \
  build-essential \
  pkg-config \
  lbzip2 \
  libprotobuf-dev \
  libsqlite3-dev \
  protobuf-compiler \
  patch \
  rsync \
  wget

# Install vim as text editor
RUN apt-get install -y vim

RUN apt-get install -y nano

RUN apt-get install -y xterm

RUN apt install -y default-jdk

# Install SUMO
RUN add-apt-repository ppa:sumo/stable

RUN apt-get update

RUN apt-get install -y \
    sumo \
    sumo-tools \
    sumo-doc

RUN adduser $SUMO_USER --disabled-password

# Enable sudo
RUN apt-get update && apt-get install -y sudo

RUN adduser $SUMO_USER sudo --disabled-password

COPY traci_update/connection.py ./usr/share/sumo/tools/traci/
COPY traci_update/constants.py ./usr/share/sumo/tools/traci/
COPY traci_update/main.py ./usr/share/sumo/tools/traci/

RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> \
/etc/sudoers


### Docker (Optional) ###
# RUN apt-get update && \
#     apt-get -qy full-upgrade && \
#     apt-get install -qy curl && \
#     apt-get install -qy curl && \
#     curl -sSL https://get.docker.com/ | sh

# RUN adduser $SUMO_USER docker --disabled-password
