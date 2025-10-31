#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Allow local docker access to X server
# xhost +local:docker

# Ensure access is revoked on script exit
# trap 'xhost -local:docker' EXIT

# Run script in the directory of the script
(
cd "$SCRIPT_DIR"

LOCKFILE=".docker_xhost.lock"
COUNTFILE=".docker_xhost"

# Acquire lock briefly just to update the count
{
    exec 9>"$LOCKFILE"
    flock -x 9  # exclusive lock (blocking)
    
    # Read current count
    if [[ -f "$COUNTFILE" ]]; then
        count=$(<"$COUNTFILE")
    else
        count=0
    fi

    ((count++))
    if (( count == 1 )); then
        echo "Enabling xhost for docker..."
        xhost +local:docker
    fi

    echo "$count" > "$COUNTFILE"

    # Unlock immediately
    flock -u 9
    exec 9>&-
}

# Define cleanup function
cleanup() {
    {
        exec 9>"$LOCKFILE"
        flock -x 9  # lock again to safely modify count

        if [[ -f "$COUNTFILE" ]]; then
            count=$(<"$COUNTFILE")
            ((count--))
            if (( count <= 0 )); then
                echo "Disabling xhost for docker..."
                xhost -local:docker
                rm -f "$COUNTFILE"
            else
                echo "$count" > "$COUNTFILE"
            fi
        fi

        flock -u 9
        exec 9>&-

        rm -f "$LOCKFILE"
    }
}
trap cleanup EXIT

if [[ "$1" == "build" ]]; then
    docker compose -f compose.yaml up -d --build
elif [[ "$1" == "enter" ]]; then
    docker exec -it liloc_ros2 /bin/bash
else
    docker compose -f compose.yaml up -d
    docker exec -it liloc_ros2 /bin/bash
fi
)