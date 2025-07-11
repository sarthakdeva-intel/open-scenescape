#!/bin/bash

FLAMEGRAPH_DIR=/home/labrat/tdorau/repos/FlameGraph

DEFAULT_PROFILE="on-host-cgroup"
# Profile can be one of:
# - on-host-cgroup: profile the container using cgroup
# - on-host-by-pid: profile the host process by PID
# - in-container: profile the container process directly
PROFILE=${1:-$DEFAULT_PROFILE}

# can be one of: fp, dwarf, lbr
UNWIND_METHOD=${2:-fp}

: "${DURATION:=10}"
: "${SAMPLING_FREQ:=99}"

pushd $(pwd)

CONTAINER_ID=$(docker ps -q --filter "name=scenescape-scene-1")
FULL_CONTAINER_ID=$(docker ps --no-trunc | grep ${CONTAINER_ID} | awk '{print $1 }')
HOST_PID=$(echo $(pgrep -f "python3 /home/scenescape/SceneScape/controller-cmd") | tr -d '\r')
CONTAINER_PID=$(echo $(docker exec -it ${CONTAINER_ID} pgrep -f python3) | tr -d '\r')

# summary
summary() {
    echo "Profile Name: ${PROFILE}"
    echo "Unwind Method: ${UNWIND_METHOD}"
    echo "Duration: ${DURATION} seconds"
    echo "Sampling Frequency: ${SAMPLING_FREQ} Hz"
    echo "Container ID: ${CONTAINER_ID}"
    echo "Full Container ID: ${FULL_CONTAINER_ID}"
    docker inspect ${CONTAINER_ID} | grep -i privileged
    echo "Host PID: ${HOST_PID}"
    echo "Container PID: ${CONTAINER_PID}"
    echo "Container Python version: $(docker exec -it ${CONTAINER_ID} python --version)"
    echo "Container Python HAVE_PERF_TRAMPOLINE: $(docker exec -it ${CONTAINER_ID} python -m sysconfig | grep HAVE_PERF_TRAMPOLINE)"
    echo "Host Python version: $(python --version)"
    echo "Host Python HAVE_PERF_TRAMPOLINE: $(docker exec -it ${CONTAINER_ID} python -m sysconfig | grep HAVE_PERF_TRAMPOLINE)"
}

OUTDIR="perf-out-${PROFILE}-$(date '+%Y-%m-%d_%H-%M-%S')"
mkdir -p "$OUTDIR"
cd "$OUTDIR"

# perf profiles
on-host-cgroup() {
    echo "Type Ctrl+C to stop profiling..."
    sudo perf record -F ${SAMPLING_FREQ} --call-graph ${UNWIND_METHOD} -e cpu-clock --cgroup system.slice/docker-${FULL_CONTAINER_ID}.scope -g
    docker exec ${CONTAINER_ID} mv /tmp/perf-${CONTAINER_PID}.map /home/scenescape/SceneScape/media
    docker run -v ./:/output -v scenescape_vol-media:/input alpine mv /input/perf-${CONTAINER_PID}.map /output/perf-${HOST_PID}.map
    sudo cp perf-${HOST_PID}.map /tmp
}

on-host-by-pid() {
    echo "Profiling for ${DURATION} seconds..."
    sudo perf record -F 1${SAMPLING_FREQ} -p ${HOST_PID} -g -- sleep ${DURATION}
    docker exec ${CONTAINER_ID} mv /tmp/perf-${CONTAINER_PID}.map /home/scenescape/SceneScape/media
    docker run -v ./:/output -v scenescape_vol-media:/input alpine mv /input/perf-${CONTAINER_PID}.map /output/perf-${HOST_PID}.map
    sudo cp perf-${HOST_PID}.map /tmp
}

in-container() {
    # in-container-${UNWIND_METHOD}:
    echo "Profiling for ${DURATION} seconds..."
    docker exec -t ${CONTAINER_ID} perf record -F ${SAMPLING_FREQ} -p ${CONTAINER_PID} -g --call-graph ${UNWIND_METHOD} -- sleep ${DURATION}
    docker exec ${CONTAINER_ID} mv perf.data /home/scenescape/SceneScape/media/perf.data
    docker run -v ./:/output -v scenescape_vol-media:/input alpine mv /input/perf.data /output
}

summary
eval ${PROFILE}

echo "Profiling data saved to: $OUTDIR/perf.data"
sudo perf script > out.perf
sudo $FLAMEGRAPH_DIR/stackcollapse-perf.pl out.perf > out.folded
sudo $FLAMEGRAPH_DIR/flamegraph.pl out.folded > flamegraph.svg
echo "Results written to output directory: $OUTDIR"
sudo chown labrat:labrat ./*

perf report -n --stdio

popd
