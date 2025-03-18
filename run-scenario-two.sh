#!/bin/bash
#set -x


echo "Quality of Service use case"
simTime=1.1 # simulation time
configuration=0 # 0: NR carrier at 850 MHz, low traffic | 1: NR carrier at 3.5 GHz, low traffic | 2: NR carrier at 28 GHz, high traffic
PercUEeMBB=0.4
PercUEURLLC=0.3
ues=1 # Number of UEs for each mmWave ENB
useSemaphores=0
controlFileName="" # QoS control file path

# Select 0 or 1 to switch between the optimized or debug build
build=0
builf_conf=0

if [[ build -eq 1 ]];then
  if [[ build_conf -eq 0 ]];then
    # Debug build
    echo "Build ns-3 in debug mode"
    ./ns3 configure --build-profile=debug --out=build/debug
  else
      # Optimized build
    echo "Build ns-3 in optimized mode"
      ./ns3 configure --build-profile=optimized --out=build/optimized
  fi
fi

./ns3 run "scratch/scenario-two --RngRun=4200\
                                    --simTime=$simTime \
                                    --PercUEeMBB=$PercUEeMBB \
                                    --PercUEURLLC=$PercUEURLLC \
                                    --configuration=$configuration \
                                    --ues=$ues \
                                    --useSemaphores=$useSemaphores \
                                    --controlFileName=$controlFileName" > output.txt 2>&1;
