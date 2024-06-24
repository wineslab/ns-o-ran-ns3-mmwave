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
simTime=10 # simulation time
e2TermIp="10.102.157.65" # actual E2term IP interface
rlcAmEnabled="true"
bufferSize=10
numberOfRaPreambles=30
heuristicType=0 # Type of heuristic for managing BS status: no heuristic (-1), Random sleeping (0), Static sleeping (1), Dynamic sleeping (2), Random action (3)
#heuristic parameters
probOn=0.6038
probIdle=0.3854
probSleep=0.0107
probOff=0.0
sinrTh=73.0
bsOn=4
bsIdle=3
bsSleep=3
bsOff=3

# Useful parameters to be configured
seed=555 # seed parameter to be used
basicCellId=1 # The next value will be the first cellId
reducedPmValues=0 # use reduced subset of pmValues
EnableE2FileLogging=1 # enable offline generation of data
ues=7 # Number of UEs for each mmWave ENB
dataRate=0

# Select 0 or 1 to switch between the optimized or debug build
# build=0
# if [[ build -eq 0 ]];then
#   if [[ build_conf -eq 0 ]];then
#     # Debug build
#     echo "Build ns-3 in debug mode"
#     ./ns3 configure --build-profile=debug --enable-examples --enable-tests --out=build/debug
#   else
#       # Optimized build
#     echo "Build ns-3 in optimized mode"
#       ./ns3 configure --build-profile=optimized --enable-examples --enable-tests --out=build/optimized
#   fi
# fi

## Energy Efficiency use case
echo "Energy Efficiency use case"
outageThreshold=-5.0 # use -5.0 when handover is not in NoAuto 
handoverMode="DynamicTtt"
indicationPeriodicity=0.02 # value in seconds (20 ms)
controlFileName="" # ES control file path

scheduleControlMessages=0 # if the control message shall be read at the beginning of the simulation and the events scheduled
# If scheduleControlMessages is 0, remember to create an empty version of the control file before the start of this script, otherwise it would lead to premature crashes.

# NS_LOG="KpmIndication"
# NS_LOG="RicControlMessage" 

for i in $(seq 1 $N); do
  echo "Running simulation $i out of $N";
  ./ns3 run "scratch/scenario-five --RngRun=$i \
                                    --configuration=$configuration \
                                    --rlcAmEnabled=$rlcAmEnabled \
                                    --bufferSize=$bufferSize \
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
                                    --controlFileName=$controlFileName\
                                    --heuristicType=$heuristicType\
                                    --probOn=$probOn\
                                    --probIdle=$probIdle\
                                    --probSleep=$probSleep\
                                    --probOff=$probOff\
                                    --sinrTh=$sinrTh\
                                    --bsOn=$bsOn\
                                    --bsIdle=$bsIdle\
                                    --bsSleep=$bsSleep\
                                    --bsOff=$bsOff\
                                    --scheduleControlMessages=$scheduleControlMessages";
  sleep 1;
done
