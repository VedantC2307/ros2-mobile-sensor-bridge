# Use ROS2 Humble with desktop-full as base image
FROM osrf/ros:humble-desktop-full-jammy

# Set non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    gnupg \
    lsb-release \
    openssl \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install Node.js and npm (Setting up the repository for the latest LTS version)
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y nodejs \
    && npm install -g npm@latest

# Create workspace directory
RUN mkdir -p /ros2_ws/src/mobile_sensor

# Set working directory
WORKDIR /ros2_ws

# Copy the mobile_sensor package code 
# This assumes you'll build the image with the mobile_sensor code in the build context
COPY . /ros2_ws/src/mobile_sensor/

# Install Node.js dependencies during build
RUN cd /ros2_ws/src/mobile_sensor && npm install

# Generate SSL certificates during build
RUN cd /ros2_ws/src/mobile_sensor/src && \
    if [ -f "generate_ssl_cert.sh" ]; then \
      chmod +x generate_ssl_cert.sh && \
      ./generate_ssl_cert.sh; \
    else \
      mkdir -p ssl && \
      openssl req -x509 -newkey rsa:4096 -keyout ssl/key.pem -out ssl/cert.pem -days 365 -nodes -subj "/CN=localhost"; \
    fi

# Build the package - explicitly using bash for the source command
RUN bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select mobile_sensor"

# Expose the web server port
EXPOSE 4000

# Set up the simplified entrypoint
COPY ./docker/docker-entrypoint.sh /
RUN chmod +x /docker-entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/docker-entrypoint.sh"]