#!/bin/bash

FLAMEGRAPH_DIR=/home/labrat/tdorau/repos/FlameGraph
PATH=$PATH:$FLAMEGRAPH_DIR

PROFILE_NAME=$1
# can be one of: fp, dwarf, lbr
UNWIND_METHOD=${2:-fp}

: "${DURATION:=10}"
: "${SAMPLING_FREQ:=99}"

pushd $(pwd)

CONTAINER_ID=$(docker ps -q --filter "name=scenescape-scene-1")
FULL_CONTAINER_ID=$(docker ps --no-trunc | grep ${CONTAINER_ID} | awk '{print $1 }')
HOST_PID=$(echo $(pgrep -f "python3 /home/scenescape/SceneScape/controller-cmd") | tr -d '\r') #$(docker inspect --format '{{.State.Pid}}' ${CONTAINER_ID})
CONTAINER_PID=$(echo $(docker exec -it ${CONTAINER_ID} pgrep -f python3) | tr -d '\r')

# summary
summary() {
    echo "Profile Name: ${PROFILE_NAME}"
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

OUTDIR="perf-out-$PROFILE_NAME-$(date '+%Y-%m-%d_%H-%M-%S')"
mkdir -p "$OUTDIR"
cd "$OUTDIR"

# perf commands
on-host-cgroup() {
    echo "Type Ctrl+C to stop profiling..."
    echo "Profiling for ${DURATION} seconds..."
    sudo perf record -F ${SAMPLING_FREQ} --call-graph ${UNWIND_METHOD} -e cpu-clock --cgroup system.slice/docker-${FULL_CONTAINER_ID}.scope -g
}

on-host-by-pid() {
    sudo perf record -F 1${SAMPLING_FREQ} -p ${HOST_PID} -g -- sleep ${DURATION}
}

in-container() {
    # in-container-${UNWIND_METHOD}:
    docker exec -t ${CONTAINER_ID} perf record -F ${SAMPLING_FREQ} -p ${CONTAINER_PID} -g --call-graph ${UNWIND_METHOD} -- sleep ${DURATION}
    docker exec ${CONTAINER_ID} mv perf.data /home/scenescape/SceneScape/media/perf.data
    docker run -v ./:/output -v scenescape_vol-media:/input alpine mv /input/perf.data /output
}

summary
# on-host-cgroup
on-host-by-pid

sudo chown labrat:labrat perf.data
echo "Profiling data saved to: $OUTDIR/perf.data"
perf script > out.perf
stackcollapse-perf.pl out.perf > out.folded
flamegraph.pl out.folded > flamegraph.svg
echo "Results written to output directory: $OUTDIR"
perf report -n --stdio

popd
