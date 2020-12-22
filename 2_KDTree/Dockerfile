# Copyright (C) 2020  Leo Wang
# https://github.com/LYON-WANG/Learning_Open3D
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Part to run Open3D C++ examples in Docker.

FROM ubuntu:18.04 as builder
ENV DEBIAN_FRONTEND=noninteractive 

ADD . /opt/sources
WORKDIR /opt/sources

RUN apt-get update && \
    apt-get install -y \
    build-essential \
    software-properties-common \
    unzip \
    git \
    apt-utils \
    wget

RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null && \
    apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'
RUN apt-get update && \
    apt-get install -y cmake

RUN apt-get update
RUN apt-get install -y libeigen3-dev \
    xorg-dev \
    libglu1-mesa-dev \
    python3-dev \
    libsdl2-dev \
    libc++-7-dev \
    libc++abi-7-dev \
    ninja-build \
    libxi-dev \
    libtbb-dev \
    libosmesa6-dev \
    libusb-1.0-0-dev

RUN git clone --recursive https://github.com/intel-isl/Open3D && \
    cd Open3D && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && make install

WORKDIR /opt/sources

RUN mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/tmp/dest .. && \
    make

ENTRYPOINT ["/usr/bin/KDTree"]
