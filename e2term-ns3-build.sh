#!/bin/bash
# set -x

echo "NS3-E2SIM Toolchain builder"

# log_level = 0 ->  LOG_LEVEL_UNCOND   0
# log_level = 1 -> LOG_LEVEL_ERROR     1
# log_level = 2 -> LOG_LEVEL_INFO      2
# log_level = 3 -> LOG_LEVEL_DEBUG     3

log_level=$1

if [ -n "${log_level}" ]; then
  echo "Setting custom log level to ${log_level}"
else
  log_level=2
  echo "Using default log level (${log_level})"
fi

cd ../oran-e2sim/e2sim

./build_e2sim.sh ${log_level}

echo "Starting build of NS3"

 cd ../../ns3-mmwave-oran/

 ./waf clean
 ./waf build

 echo "Build of NS3 completed, you can now use the ./run_simulations.sh script"
 echo
 echo 