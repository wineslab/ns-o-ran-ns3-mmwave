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
simTime=0.8 # simulation time
e2TermIp="10.102.157.65" # actual E2term IP interface
rlcAmEnabled="true"
bufferSize=10
trafficModel=0
numberOfRaPreambles=40

heuristicType=2 # Type of heuristic for managing BS status: no heuristic (-1), Random sleeping (0), Static sleeping (1), Dynamic sleeping (2)
#heuristic parameters
probOn=0.5
probIdle=0.0
probSleep=0.0
probOff=0.5
sinrTh=73.0
bsOn=4
bsIdle=3
bsSleep=3
bsOff=3

# Useful parameters to be configured
basicCellId=1 # The next value will be the first cellId
reducedPmValues=0 # use reduced subset of pmValues
EnableE2FileLogging=1 # enable offline generation of data
ues=2 # Number of UEs for each mmWave ENB
dataRate=0
hoSinrDifference=3

# # Select 0 or 1 to switch between the optimized or debug build
# build=1
# builf_conf=0

# if [[ build -eq 0 ]];then
#   if [[ build_conf -eq 0 ]];then
#     # Debug build
#     echo "Build ns-3 in debug mode"
#     ./waf configure --build-profile=debug --out=build/debug
#   else
#       # Optimized build
#     echo "Build ns-3 in optimized mode"
#       ./waf configure --build-profile=optimized --out=build/optimized
#   fi
# fi

## Energy Efficiency use case
echo "Energy Efficiency use case"
outageThreshold=-5.0 # use -5.0 when handover is not in NoAuto 
handoverMode="DynamicTtt"
indicationPeriodicity=0.1 #0.02 value in seconds (20 ms)
controlFileName="es_actions_for_ns3.csv" # ES control file path

#scheduleControlMessages=1 # if the control message shall be read at the beginning of the simulation and the events scheduled
# If scheduleControlMessages is 0, remember to create an empty version of the control file before the start of this script, otherwise it would lead to premature crashes.

# NS_LOG="KpmIndication"
# NS_LOG="RicControlMessage" 

./ns3 run --command-template="valgrind --leak-check=full --show-reachable=yes --num-callers=50 --track-origins=yes --xml=yes --xml-file=valgrind_output.xml %s \
                                    --configuration=$configuration \
                                    --hoSinrDifference=$hoSinrDifference \
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
                                    --numberOfRaPreambles=$numberOfRaPreambles \
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
                                    --bsOff=$bsOff" scenario-three > log.out 2>&1;
