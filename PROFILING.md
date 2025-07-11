# Performance Profiling Guide

This guide explains how to profile the SceneScape application using Linux perf tools and FlameGraph visualization.

## Useful Sources

- [Brendan Gregg's perf page](https://www.brendangregg.com/perf.html)
- [Brendan Gregg's page about flamegraphs](https://www.brendangregg.com/flamegraphs.html)
- [Python perf profiling documentation (3.12)](https://docs.python.org/3.12/howto/perf_profiling.html)
- [Python perf profiling documentation (3.x)](https://docs.python.org/3/howto/perf_profiling.html)
- [Brendan Gregg's talk at Netflix](https://www.youtube.com/watch?v=UVM3WX8Lq2k) - general understanding of flamegraphs, profiling challenges and how perf can help
- [Kernel Recipes 2017 slides](https://www.slideshare.net/brendangregg/kernel-recipes-2017-using-linux-perf-at-netflix#31)
- [Making flamegraphs with containerized Java](https://blog.alicegoldfuss.com/making-flamegraphs-with-containerized-java/) - the same hack is used here for Python

## Limitations

The script and the code on this branch is designed to profile specifically the Controller service, so that both C++ and Python stacks are observable.

The subsequent profiling runs for the same container may not produce good results as the first run. Therefore, it is suggested to restart the service before next profiling run.

For `on-host-cgroup` profile the duration is decided by the user who stops profiling with Ctrl+C, so DURATION variable has no effect.

## Prerequisites

1. **Install perf tools:**
   ```bash
   sudo apt install linux-tools-common linux-tools-generic linux-tools-$(uname -r)
   ```

2. **Install FlameGraph:**
   ```bash
   git clone https://github.com/brendangregg/FlameGraph.git
   ```

3. **Adjust FlameGraph directory path:**
   - Update the `FLAMEGRAPH_DIR` variable in `run-profiling.sh` to point to your FlameGraph directory

4. **Enable CPU profiling:**
   ```bash
   sudo sysctl -w kernel.perf_event_paranoid=-1
   ```

## Profiling Steps

1. **Restart the scene service:**
   - Restart the scenescape container or the whole scenescape application

2. **Run the profiling script:**
   ```bash
   ./run-profiling.sh [PROFILE] [UNWIND_METHOD]
   ```

   You can try different profiling methods:
   - `on-host-cgroup`: Profile the container using cgroup (default)
   - `on-host-by-pid`: Profile the host process by PID
   - `in-container`: Profile the container process directly

   Available unwind methods:
   - `fp`: Frame pointer (default)
   - `dwarf`: DWARF debug info
   - `lbr`: Last Branch Record

3. **View the results:**
   - Use `sudo perf report -f -n --stdio` in the output folder to view text report
     **Note:** You need to view the results as root so that perf can read the symbols correctly
   - Open `flamegraph.svg` in a browser to inspect the flamegraph visualization

## Environment Variables

You can customize the profiling behavior using these environment variables:

- `DURATION`: Profiling duration in seconds (default: 10)
- `SAMPLING_FREQ`: Sampling frequency in Hz (default: 99)

Example:
```bash
DURATION=30 SAMPLING_FREQ=199 ./run-profiling.sh on-host-by-pid dwarf
```

## Output

The profiling script creates a timestamped output directory containing:
- `perf.data`: Raw perf data
- `out.perf`: Perf script output
- `out.folded`: Folded stack traces
- `flamegraph.svg`: Interactive flamegraph visualization
