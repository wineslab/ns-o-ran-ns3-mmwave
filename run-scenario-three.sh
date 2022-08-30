#!/bin/bash
#set -x

enableTraces=1 # enable generation of ns-3 traces 
e2lteEnabled=1 # enable e2 reports from lte macro cell
e2nrEnabled=1 # enable e2 reports from nr secondary cells
e2du=1 # enable reporting of DU PM containers
e2cuUp=1 # enable reporting of CU UP PM containers
e2cuCp=1 # enable reporting of CU CP PM containers
configuration=0 # 0: NR carrier at 850 MHz, low traffic | 1: NR carrier at 3.5 GHz, low traffic | 2: NR carrier at 28 GHz, high traffic
minSpeed=2.0 # minimum UE speed in m/s
maxSpeed=4.0 # maximum UE speed in m/s
simTime=1 # simulation time
e2TermIp="10.102.157.65" # actual E2term IP interface

# Useful parameters to be configured
N=1 # number of simulations
basicCellId=1 # The next value will be the first cellId
reducedPmValues=0 # use reduced subset of pmValues
EnableE2FileLogging=1 # enable offline generation of data
ues=3 # Number of UEs for each mmWave ENB
dataRate=0

# Select 0 or 1 to switch between the optimized or debug build
build=1
if [[ build -eq 0 ]];then
  # Debug build
   echo "Build ns-3 in debug mode"
   ./waf -debug
else
    # Optimized build
   echo "Build ns-3 in optimized mode"
    ./waf -optimized
fi

# Remember to create an empty version of the control file before the start of this script, otherwise it would lead to premature crashes.

## Energy Efficiency use case
echo "Energy Efficiency use case"
outageThreshold=-5.0 # use -5.0 when handover is not in NoAuto 
handoverMode="DynamicTtt"
indicationPeriodicity=0.02 # value in seconds (20 ms)
controlFileName="es_actions_for_ns3.csv" # ES control file path

# NS_LOG="KpmIndication"
# NS_LOG="RicControlMessage" 

for i in $(seq 1 $N); do
  echo "Running simulation $i out of $N";
  ./waf --run "scratch/scenario-three --RngRun=$i \
                                    --configuration=$configuration \
                                    --dataRate=$dataRate \
                                    --enableTraces=$enableTraces \
                                    --e2lteEnabled=$e2lteEnabled \
                                    --e2nrEnabled=$e2nrEnabled \
                                    --e2du=$e2du \
                                    --simTime=$simTime \
                                    --outageThreshold=$outageThreshold \
                                    --handoverMode=$handoverMode \
                                    --basicCellId=$basicCellId \
                                    --e2cuUp=$e2cuUp \
                                    --e2cuCp=$e2cuCp \
                                    --ues=$ues \
                                    --reducedPmValues=$reducedPmValues \
                                    --e2TermIp=$e2TermIp \
                                    --enableE2FileLogging=$EnableE2FileLogging \
                                    --minSpeed=$minSpeed\
                                    --maxSpeed=$maxSpeed\
                                    --indicationPeriodicity=$indicationPeriodicity\
                                    --controlFileName=$controlFileName";
  sleep 1;
done
