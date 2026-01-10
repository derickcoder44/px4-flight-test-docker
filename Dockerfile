FROM ghcr.io/derickcoder44/px4-sim-docker:latest

ENV DEBIAN_FRONTEND=noninteractive

# Install video recording dependencies
RUN apt-get update && apt-get install -y \
    ffmpeg \
    x11-utils \
    xvfb \
    xdotool \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglvnd0 \
    libglx0 \
    libegl1 \
    && rm -rf /var/lib/apt/lists/*

# Copy flight test scripts
COPY scripts/flight_test.py /root/scripts/
COPY scripts/run_flight_test.sh /root/scripts/
RUN chmod +x /root/scripts/*.sh /root/scripts/*.py

WORKDIR /root/workspace

# Update entrypoint
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
source /root/workspace/ros2_ws/install/setup.bash\n\
export PATH="/root/scripts:$PATH"\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
