# Multi-stage Docker image for SENTIO CI/CD

FROM ros:rolling-ros-core-jammy AS base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=rolling

# Install base dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    curl \
    wget \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Install Node.js 20
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get install -y nodejs && \
    npm install -g npm@latest

# Install Python development tools
RUN pip install --no-cache-dir \
    black \
    flake8 \
    isort \
    mypy \
    pytest \
    pytest-cov \
    pytest-xdist \
    pyyaml \
    platformio

# Build stage
FROM base AS build

WORKDIR /sentio

# Copy source
COPY . .

# Build ROS packages
RUN . /opt/ros/rolling/setup.sh && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths sentio_ros2_ws/src --ignore-src -y && \
    cd sentio_ros2_ws && \
    colcon build --symlink-install

# Runtime stage
FROM base AS runtime

WORKDIR /sentio

# Copy built packages from build stage
COPY --from=build /sentio/sentio_ros2_ws/install /sentio/sentio_ros2_ws/install
COPY --from=build /sentio/sentio_ros2_ws/build /sentio/sentio_ros2_ws/build

# Copy source
COPY . .

# Setup entry point
RUN echo '#!/bin/bash' > /entrypoint.sh && \
    echo 'source /opt/ros/rolling/setup.bash' >> /entrypoint.sh && \
    echo 'source /sentio/sentio_ros2_ws/install/setup.bash' >> /entrypoint.sh && \
    echo 'exec "$@"' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

# Labels
LABEL org.opencontainers.image.title="SENTIO"
LABEL org.opencontainers.image.description="SENTIO Robot - CI/CD Image"
LABEL org.opencontainers.image.vendor="DevSora Deep-Tech Research"
