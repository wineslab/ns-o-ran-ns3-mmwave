#!/bin/bash
#set -x

enableTraces=1 # enable generation of ns-3 traces 
e2lteEnabled=1 # enable e2 reports from lte macro cell
e2nrEnabled=1 # enable e2 reports from nr secondary cells
e2du=1 # enable reporting of DU PM containers
e2cuUp=1 # enable reporting of CU UP PM containers
e2cuCp=1 # enable reporting of CU CP PM containers
trafficModel=3 # Type of the traffic model at the transport layer [0,3], can generate full buffer traffic (0), half nodes in full buffer and half nodes in bursty (1), bursty traffic (2), mixed setup (3)
configuration=0 # 0: NR carrier at 850 MHz, low traffic | 1: NR carrier at 3.5 GHz, low traffic | 2: NR carrier at 28 GHz, high traffic
minSpeed=2.0 # minimum UE speed in m/s
maxSpeed=4.0 # maximum UE speed in m/s
simTime=1.0 # simulation time
e2TermIp="10.102.157.65" # actual E2term IP interface
ueZeroPercentage=-1 # PDCP split for UE RNTI 0 on eNB

# Useful parameters to be configured
N=1 # number of simulations
basicCellId=1 # The next value will be the first cellId
reducedPmValues=0 # use reduced subset of pmValues
EnableE2FileLogging=1 # enable offline generation of data
ues=3 # Number of UEs for each mmWave ENB

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

# Select 0 or 1 to switch between the use cases
# Remember to create an empty version of the control file before the start of this script, otherwise it would lead to premature crashes.
use_case=1
if [[ use_case -eq 0 ]];then
  ## Energy Efficiency use case
  echo "Energy Efficiency use case"
  outageThreshold=-5.0 # use -5.0 when handover is not in NoAuto 
  handoverMode="DynamicTtt"
  indicationPeriodicity=0.02 # value in seconds (20 ms)
  controlPath="es_actions_for_ns3.csv" # EE control file path
  numberOfRaPreambles=20
elif [[ use_case -eq 1 ]];then
  ## Traffic Steering use case
  echo "Traffic Steering use case"
  outageThreshold=-1000
  handoverMode="NoAuto"
  indicationPeriodicity=0.1 # value in seconds (100 ms)
  controlPath="ts_actions_for_ns3.csv" # TS control file path
  numberOfRaPreambles=40
else
  ## Quality of Service use case
  echo "Quality of Service use case"
  outageThreshold=-5.0 # use -5.0 when handover is not in NoAuto 
  handoverMode="DynamicTtt"
  indicationPeriodicity=0.02 # value in seconds (20 ms)
  # controlPath="qos_actions.csv" # QoS control file path, decomment for control
  # ueZeroPercentage=0.1
  numberOfRaPreambles=40
fi

#  NS_LOG="LteEnbNetDevice:LteEnbRrc:LteUeRrc:McEnbPdcp:McUePdcp" 
# ./ns3 configure --enable-eigen --build-profile=optimized --out=build/optimized

for i in $(seq 1 $N); do
  echo "Running simulation $i out of $N";
  ./ns3 run "scratch/scenario-one --RngRun=$i \
                                    --configuration=$configuration \
                                    --trafficModel=$trafficModel \
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
                                    --numberOfRaPreambles=$numberOfRaPreambles\
                                    --indicationPeriodicity=$indicationPeriodicity\
                                    --controlFileName=$controlFileName";
  sleep 1;
done
