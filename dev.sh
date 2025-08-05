#!/bin/bash
# ----------------------------------------------------------------
# Build docker dev stage and add local code for live development
# ----------------------------------------------------------------

CYCLONE_VOL=""

# Default cyclone_dds.xml path
CYCLONE_DIR=/home/$USER/cyclone_dds.xml
# Default in-vehicle rosbags directory
ROSBAGS_DIR=/recorded_datasets/edinburgh
# Default export directory
EXPORT_DIR=/mnt/vdb/exported_data
# Default value for headless
headless=false

# Function to print usage
usage() {
    echo "
Usage: dev.sh [-l|--local] [--path | -p ] [--output | -o ] [--headless] [--help | -h]

Options:
    -l | --local    Use default local cyclone_dds.xml config
                    Optionally point to absolute -l /path/to/cyclone_dds.xml
    -p | --path     ROSBAGS_DIR_PATH
                    Specify path to store recorded rosbags
    -o | --output   EXPORT_DIR
                    Specify path where exported data is stored
    --headless      Run the Docker image without X11 forwarding
    -h | --help     Display this help message and exit.
    "
    exit 1
}

# Parse command-line options
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -l|--local)
            if [[ -n "$2" && "$2" != -* ]]; then
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
        -o|--output)
            if [[ -n "$2" && "$2" != -* ]]; then
                EXPORT_DIR="$2"
                shift
            else
                echo "Error: Argument for $1 is missing."
                usage
            fi
            ;;
        --headless) headless=true ;;
        -h|--help) usage ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
    shift
done


# Verify CYCLONE_DIR exists
if [ -n "$CYCLONE_VOL" ]; then
    if [ ! -f "$CYCLONE_DIR" ]; then
        echo "$CYCLONE_DIR does not exist! Please provide a valid path to cyclone_dds.xml"
        exit 1
    fi
fi

# Verify ROSBAGS_DIR exists
if [ ! -d "$ROSBAGS_DIR" ]; then
    echo "$ROSBAGS_DIR does not exist! Please provide a valid path to store rosbags"
    exit 1
fi

# Verify EXPORT_DIR exists
if [ ! -d "$EXPORT_DIR" ]; then
    echo "$EXPORT_DIR does not exist! Please provide a valid path where exported data is stored"
    exit 1
fi


MOUNT_X=""
if [ "$headless" = "false" ]; then
    MOUNT_X="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix"
    xhost + >/dev/null
fi

# Build docker image up to dev stage
docker build \
    --build-arg USER_ID=$(id -u) \
    --build-arg GROUP_ID=$(id -g) \
    --build-arg USERNAME=rosbag_expoter \
    -t tartan_rosbag_expoter:latest-dev \
    -f Dockerfile --target dev .

# Get the absolute path of the script
SCRIPT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")

# Run docker image with local code volumes for development
docker run -it --rm --net host --privileged \
    --user "$(id -u):$(id -g)" \
    ${MOUNT_X} \
    -e XAUTHORITY="${XAUTHORITY}" \
    -e XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
    -v /dev:/dev \
    -v /tmp:/tmp \
    $CYCLONE_VOL \
    -v $ROSBAGS_DIR:/opt/ros_ws/rosbags \
    -v $EXPORT_DIR:/opt/ros_ws/exported_data \
    -v $SCRIPT_DIR/ros2_bag_exporter:/opt/ros_ws/src/ros2_bag_exporter \
    -v /etc/localtime:/etc/localtime:ro \
    tartan_rosbag_expoter:latest-dev
