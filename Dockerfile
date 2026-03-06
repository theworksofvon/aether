FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /aether

RUN pip install --no-cache-dir --break-system-packages \
    adafruit-circuitpython-dht \
    adafruit-blinka \
    lgpio

COPY . .



RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
ENV LGPIO_CHIP=4

CMD ["/bin/bash"]