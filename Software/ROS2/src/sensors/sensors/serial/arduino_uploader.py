import subprocess
import os
import argparse

# NOTE: You need to have arduino-cli installed and configured on your system.
# On Linux, install it with
# curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

def upload_sketch(port: str, sketch_name: str, is_nano: bool, old_nano: bool = False):
    """
    Compile and upload an Arduino sketch using arduino-cli.

    Parameters:
    - port: Serial port (e.g., '/dev/ttyUSB0')
    - sketch_name: Full path to the sketch .ino file (e.g., '~/NULunabotics/Software/ESP8266/Sketch/Sketch.ino')
    - is_nano: True for Arduino Nano, False for ESP8266
    """
    # Expand the ~ in paths
    sketch_path = os.path.expanduser(sketch_name)
    sketch_dir = os.path.dirname(sketch_path)

    if is_nano:
        if old_nano:
            fqbn = "arduino:avr:nano:cpu=atmega328old"
        else:
            fqbn = "arduino:avr:nano"
        additional_flags = []
        board_name = "Arduino Nano"
    else:
        fqbn = "esp8266:esp8266:nodemcuv2"
        additional_flags = []
        board_name = "ESP8266"

    try:
        # Step 1: Compile the sketch
        print(f"Compiling sketch in {sketch_dir} for {board_name}...")
        subprocess.run(["arduino-cli", "compile", "--fqbn", fqbn, sketch_dir], check=True)

        # Step 2: Upload the compiled binary
        print(f"Uploading to port {port}...")
        subprocess.run(
            ["arduino-cli", "upload", "-p", port, "--fqbn", fqbn, sketch_dir] + additional_flags,
            check=True
        )
        print("✅ Upload successful.")
    except subprocess.CalledProcessError as e:
        print("❌ Error during upload process:", e)


def main(args=None):

    parser = argparse.ArgumentParser(description="Upload Arduino sketch.")
    parser.add_argument("port", type=str, help="Serial port (e.g., '/dev/ttyUSB0')")
    parser.add_argument("sketch_name", type=str, help="Full path to the sketch .ino file")
    parser.add_argument("--nano", action="store_true", help="Specify if the board is Arduino Nano")
    parser.add_argument("--old", action="store_true", help="Specify if the board is an old Arduino Nano")
    args = parser.parse_args(args)

    upload_sketch(args.port, args.sketch_name, args.nano, args.old)

# example command to run the script:
# python3 arduino_uploader.py /dev/ttyUSB0 Motors --nano

if __name__ == "__main__":
    main()
