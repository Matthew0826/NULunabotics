#!/bin/bash

# Source and destination paths
SOURCE_DIR="$(cd "$(dirname "$0")"; pwd)"  # Folder where the script is
DEST_DIR="$HOME/Arduino/libraries/TheiaSerial"

# Create the destination directory if it doesn't exist
mkdir -p "$DEST_DIR"

# Loop through all files/folders in the source directory
for item in "$SOURCE_DIR"/*; do
    # Skip this script itself
    if [[ "$item" == "$SOURCE_DIR/$(basename "$0")" ]]; then
        continue
    fi

    # Create symlink in the destination
    ln -sf "$item" "$DEST_DIR/"
done

echo "Linked all files from $SOURCE_DIR to $DEST_DIR."
