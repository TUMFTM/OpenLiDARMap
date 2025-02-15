FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y software-properties-common locales && \
    add-apt-repository universe
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Create a non-root user
ARG USERNAME=local
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [Optional] Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

RUN apt-get update && apt-get install -y -q --no-install-recommends \
    git \
    vim \
    build-essential \
    cmake \
    make \
    gdb \
    wget \
    libeigen3-dev \
    libomp-dev \
    libtbb-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-distutils \
    pybind11-dev \
    liblzf-dev

RUN pip install numpy ninja

WORKDIR /dev_ws

# Ceres
RUN wget http://ceres-solver.org/ceres-solver-2.2.0.tar.gz && \
    tar zxf ceres-solver-2.2.0.tar.gz && \
    mkdir ceres-bin && cd ceres-bin && \
    cmake ../ceres-solver-2.2.0 && make -j3 && make test && make install

# Irindescence
RUN apt-get update && apt-get install -y -q --no-install-recommends curl gnupg libpng-dev libjpeg-dev
RUN mkdir -m 0755 -p /etc/apt/keyrings/
RUN curl -fsSL https://koide3.github.io/ppa/ubuntu2204/KEY.gpg | gpg --dearmor -o /etc/apt/keyrings/koide3_ppa.gpg
RUN echo "deb [signed-by=/etc/apt/keyrings/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2204 ./" | tee /etc/apt/sources.list.d/koide3_ppa.list > /dev/null
RUN chmod 644 /etc/apt/keyrings/koide3_ppa.gpg && chmod 644 /etc/apt/sources.list.d/koide3_ppa.list
RUN apt-get update && apt-get install -y -q --no-install-recommends libiridescence-dev

ENV QT_DEBUG_PLUGINS=1

COPY . /dev_ws/.

RUN mkdir -p /dev_ws/build && \
    cd /dev_ws/build && \
    cmake -DCMAKE_BUILD_TYPE=Release ../cpp/ && \
    make -j

RUN cd /dev_ws/python && \
    pip install -e .

ENV LD_LIBRARY_PATH=/usr/local/lib/python3.10/dist-packages/openlidarmap_pybind

CMD [ "bash" ]
