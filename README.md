# PX4 Flight Test Docker

Automated flight testing for PX4 drones in Gazebo simulation.

## Overview

This image builds on top of [`px4-sim-docker`](https://github.com/derickcoder44/px4-sim-docker) and adds:
- Flight test scripts (takeoff, hover, land)
- Video recording capabilities
- Automated test orchestration

## Usage

### Pull from GitHub Container Registry

```bash
docker pull ghcr.io/derickcoder44/px4-flight-test-docker:latest
```

### Run Flight Test

```bash
docker run -it --rm \
  ghcr.io/derickcoder44/px4-flight-test-docker:latest \
  /root/scripts/run_flight_test.sh
```

This will:
1. Start the Micro-XRCE-DDS Agent
2. Launch PX4 SITL with Gazebo (headless)
3. Wait for the drone to be ready
4. Execute flight test: arm → takeoff to 5m → hover for 10s → land
5. Save logs to `/root/logs/`

### Build Locally

```bash
git clone --recursive https://github.com/derickcoder44/px4-flight-test-docker.git
cd px4-flight-test-docker
docker build -t px4-flight-test-docker .
```

## Flight Test Sequence

The automated test performs:
1. **Initialization**: Start DDS agent and PX4 SITL
2. **Pre-flight**: Wait for ROS2 topics and vehicle status
3. **Arm**: Arm the vehicle
4. **Takeoff**: Ascend to 5 meters
5. **Hover**: Hold position for 10 seconds
6. **Land**: Descend and land
7. **Disarm**: Disarm the vehicle

## Architecture

This image is part of a layered Docker architecture:

1. **ros-px4-bridge-docker** (base) - ROS2 + DDS Agent + PX4 ROS packages
2. **px4-sim-docker** - Adds PX4 SITL + Gazebo
3. **px4-flight-test-docker** (this repo) - Adds flight test scripts

## Files

- `scripts/flight_test.py` - ROS2 Python node for flight control
- `scripts/run_flight_test.sh` - Orchestration script
- `Dockerfile` - Image definition
- `.github/workflows/test-flight.yml` - CI workflow

## Customization

### Modify Flight Parameters

Edit `scripts/flight_test.py`:

```python
self.takeoff_height = -5.0  # meters (NED frame, negative is up)
self.hover_duration = 10.0  # seconds
```

### Change Drone Model

Set environment variable:

```bash
docker run -e PX4_GZ_MODEL=x500_depth \
  ghcr.io/derickcoder44/px4-flight-test-docker:latest \
  /root/scripts/run_flight_test.sh
```

## CI/CD

The GitHub Actions workflow:
1. Builds the Docker image
2. Runs the flight test
3. Uploads logs as artifacts

## License

MIT
