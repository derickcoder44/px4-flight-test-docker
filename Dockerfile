FROM ghcr.io/derickcoder44/px4-sim-docker:latest

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies for headless rendering and camera recording
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-humble-cv-bridge \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglvnd0 \
    libglx0 \
    libegl1 \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for video recording
RUN pip3 install --no-cache-dir opencv-python

# Copy flight test scripts
COPY scripts/flight_test.py /root/scripts/
COPY scripts/run_flight_test.sh /root/scripts/
COPY scripts/record_camera.py /root/scripts/
RUN chmod +x /root/scripts/*.sh /root/scripts/*.py

WORKDIR /root/workspace

# Update entrypoint
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
source /root/workspace/ros2_ws/install/setup.bash\n\
export ROS_DOMAIN_ID=0\n\
export PATH="/root/scripts:$PATH"\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
