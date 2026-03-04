FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install Gazebo Harmonic
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# CPU rendering (xvfb + mesa)
RUN apt-get update && apt-get install -y \
    xvfb \
    mesa-utils \
    libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

# Python + gz-transport bindings for web viewer
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-gz-transport13 \
    python3-gz-msgs10 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install flask Pillow numpy

# Copy model and world files
COPY models/ /opt/models/
COPY worlds/ /opt/worlds/
COPY web_viewer.py /opt/web_viewer.py

# Disable shadows for CPU rendering
RUN sed -i 's/<shadows>true<\/shadows>/<shadows>false<\/shadows>/g' /opt/worlds/*.sdf 2>/dev/null; \
    sed -i 's/<cast_shadows>true<\/cast_shadows>/<cast_shadows>false<\/cast_shadows>/g' /opt/worlds/*.sdf 2>/dev/null; \
    true

ENV GZ_SIM_RESOURCE_PATH=/opt/models

EXPOSE 8081

WORKDIR /opt

# Start xvfb, gazebo (headless server), and web viewer
CMD bash -c '\
    Xvfb :1 -screen 0 1024x768x24 & \
    sleep 2 && \
    export DISPLAY=:1 && \
    gz sim -s -r /opt/worlds/xarm6_table.sdf & \
    sleep 8 && \
    python3 /opt/web_viewer.py \
'
