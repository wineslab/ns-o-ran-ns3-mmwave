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
e2TermIp="10.244.0.240" # actual E2term IP interface
basicCellId=1 # The next value will be the first cellId
ues=15 # Number of UEs for each mmWave ENB
reducedPmValues=0 # use reduced subset of pmValues
EnableE2FileLogging=1 # enable offline generation of data
controlPath="es_actions_for_ns3.csv" # full control file path

# Remove NoAuto on handover and Outage Threshold to use the Dynamic TTI HO
outageThreshold=-5.0 # use -5.0 when handover is not in NoAuto 
handoverMode="DynamicTtt"

# NS_LOG="KpmIndication"
# NS_LOG="RicControlMessage" 


./ns3 run "scratch/scenario-one-es --RngRun=18813 \
                                    --configuration=0 \
                                    --enableTraces=1 \
                                    --e2lteEnabled=1 \
                                    --e2nrEnabled=1 \
                                    --e2du=1 \
                                    --dataRate=0 \
                                    --simTime=10 \
                                    --indicationPeriodicity=0.02 \
                                    --heuristicType=0 \
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
                                    --probOn=0.8 \
                                    --probIdle=0.0 \
                                    --probSleep=0.0 \
                                    --probOff=0.2 \
                                    --sinrTh=73.0 \
                                    --numberOfRaPreambles=40\
                                    --bsOn=4 --bsIdle=3 --bsSleep=3 --bsOff=3 \
                                    --probOn=0.8 --probIdle=0.0 --probSleep=0.0 --probOff=0.2 --sinrTh=73.0 \
                                    --bsOn=2 --bsIdle=2 --bsSleep=1 --bsOff=2 \
                                    --ns3::LteEnbNetDevice::ControlFileName=''";


