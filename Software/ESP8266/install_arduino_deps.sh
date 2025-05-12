#!/bin/bash

# Installs support for:
# - Fusion library for accelerometer and gyroscope
# - SimpleVector library for vector math
# - Adafruit MPU6050 library for MPU6050 sensor
# - ESP8266 board support

# Where to put the library
LIB_DIR="$HOME/Arduino/libraries/Fusion"

# Create a temp folder and clone the repo
TMP_DIR=$(mktemp -d)
echo "Cloning Fusion library..."
git clone https://github.com/xioTechnologies/Fusion "$TMP_DIR/FusionRepo"

# Ensure destination exists
mkdir -p "$LIB_DIR"

# Copy only the Fusion folder from the repo
echo "Installing Fusion into Arduino libraries..."
cp -r "$TMP_DIR/FusionRepo/Fusion/"* "$LIB_DIR"

# Create the library.properties file
cat <<EOF > "$LIB_DIR/library.properties"
name=Fusion
version=1.0.0
author=xioTechnologies
sentence=Fusion is a sensor fusion library for Inertial Measurement Units (IMUs), optimised for embedded systems.
category=Sensor
url=https://github.com/xioTechnologies/Fusion
EOF

# Clean up
rm -rf "$TMP_DIR"

echo "Fusion library installed at $LIB_DIR"



# Other libraries to install
arduino-cli lib install SimpleVector
arduino-cli lib install "Adafruit MPU6050"

# Support for ESP8266
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://arduino.esp8266.com/stable/package_esp8266com_index.json
arduino-cli core install esp8266:esp8266
