#!/bin/bash

FLAMEGRAPH_DIR=/home/labrat/tdorau/repos/FlameGraph
PATH=$PATH:$FLAMEGRAPH_DIR

pushd $(pwd)

CONTAINER_ID=$(docker ps -q --filter "name=scenescape-scene-1")
echo "Container ID: ${CONTAINER_ID}"
docker inspect ${CONTAINER_ID} | grep -i privileged
FULL_CONTAINER_ID=$(docker ps --no-trunc | grep ${CONTAINER_ID} | awk '{print $1 }')
echo "Full Container ID: ${FULL_CONTAINER_ID}"
TARGET_PID=$(docker inspect --format '{{.State.Pid}}' ${CONTAINER_ID})
ps aux | grep ${TARGET_PID} | grep controller
echo "Target PID: ${TARGET_PID}"

# sudo /opt/intel/oneapi/vtune/latest/bin64/vtune -collect hotspots -result-dir /tmp/vtune-hotspots -target-pid=${TARGET_PID}
OUTDIR="perf-out-$(date '+%Y-%m-%d_%H-%M-%S')"
mkdir -p "$OUTDIR"
cd "$OUTDIR"

echo "Type Ctrl+C to stop profiling..."
sudo perf record -F 99 --call-graph fp -e cpu-cycles --cgroup system.slice/docker-${FULL_CONTAINER_ID}.scope -g
sudo chown labrat:labrat perf.data
echo "Profiling data saved to: $OUTDIR/perf.data"
perf script > out.perf
stackcollapse-perf.pl out.perf > out.folded
flamegraph.pl out.folded > flamegraph.svg
echo "Results written to output directory: $OUTDIR"

popd
