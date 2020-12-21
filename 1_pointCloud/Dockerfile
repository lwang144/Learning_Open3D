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

RUN apt-get update
RUN apt-get install -y \
    build-essential \
    cmake \
    software-properties-common \
    git

RUN apt-get update
RUN apt-get install -y libeigen3-dev

RUN git clone --recursive https://github.com/intel-isl/Open3D && \
    cd Open3D && \
    util/install_deps_ubuntu.sh && \
    mkdir build && \
    cd build && \
    cmake && \
    make -j$(nproc) && make install

ADD . /opt/sources
WORKDIR /opt/sources
RUN mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/tmp/dest .. && \
    make && make install

FROM ubuntu:18.04
ENV DEBIAN_FRONTEND=noninteractive 

WORKDIR /usr/bin
COPY --from=builder /tmp/dest /usr
ENTRYPOINT ["/usr/bin/pointCloud"]

