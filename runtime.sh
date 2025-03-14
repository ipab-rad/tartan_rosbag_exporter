#!/bin/bash
# ---------------------------------------------------------------------------
# Build docker image and run ROS code for runtime or interactively with bash
# ---------------------------------------------------------------------------

CYCLONE_VOL=""
BASH_CMD=()

# Default cyclone_dds.xml path
CYCLONE_DIR=/home/$USER/cyclone_dds.xml
# Default in-vehicle rosbags directory
ROSBAGS_DIR=/recorded_datasets/edinburgh
CHECK_PATH=true

# Function to print usage
usage() {
    echo "
Usage: runtime.sh [-b|bash] [-l|--local] [--path | -p ] [--help | -h] <cmd>

Options:
    -l | --local    Use default local cyclone_dds.xml config
                    Optionally point to absolute -l /path/to/cyclone_dds.xml
    -p | --path   ROSBAGS_DIR_PATH
                    Specify path to store recorded rosbags
    -h | --help     Display this help message and exit.

An interactive bash session will start if no <cmd> is provided.
    "
    exit 1
}

# Parse command-line options
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -l|--local)
            if [[ -n "$2" && "$2" != -*  && "$2" != "bash" ]]; then
                CYCLONE_DIR="$2"
                shift
            fi
            CYCLONE_VOL="-v $CYCLONE_DIR:/opt/ros_ws/cyclone_dds.xml"
            ;;
        -p|--path)
            if [[ -n "$2" && "$2" != -* ]]; then
                ROSBAGS_DIR="$2"
                shift
            else
                echo "Error: Argument for $1 is missing."
                usage
            fi
            ;;
        -h|--help)
            usage
            ;;
        *)
            BASH_CMD+=("$1")  # Save all remaining args as the command
            ;;
    esac
    shift
done

# If no command was provided, default to starting an interactive shell
if [ ${#BASH_CMD[@]} -eq 0 ]; then
    BASH_CMD=("bash")
fi

# Verify CYCLONE_DIR exists
if [ -n "$CYCLONE_VOL" ]; then
    if [ ! -f "$CYCLONE_DIR" ]; then
        echo "$CYCLONE_DIR does not exist! Please provide a valid path to cyclone_dds.xml"
        exit 1
    fi
fi

# Verify ROSBAGS_DIR exists
if [ ! -d "$ROSBAGS_DIR" -a "$CHECK_PATH" = true ]; then
    echo "$ROSBAGS_DIR does not exist! Please provide a valid path to store rosbags"
    exit 1
fi

# Build docker image only up to runtime stage
docker build \
    --build-arg USER_ID=$(id -u) \
    --build-arg GROUP_ID=$(id -g) \
    --build-arg USERNAME=rosbag_expoter \
    -t tartan_rosbag_expoter:latest \
    -f Dockerfile --target runtime .

# Get the absolute path of the script
SCRIPT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
mkdir -p $SCRIPT_DIR/output

# Run docker image
docker run -it --rm --net host --privileged \
    --user "$(id -u):$(id -g)" \
    -v /dev:/dev \
    -v /tmp:/tmp \
    $CYCLONE_VOL \
    -v $ROSBAGS_DIR:/opt/ros_ws/rosbags \
    -v $SCRIPT_DIR/output:/opt/ros_ws/output \
    -v /etc/localtime:/etc/localtime:ro \
    tartan_rosbag_expoter:latest "${BASH_CMD[@]}"
