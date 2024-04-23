#!/bin/bash
cd build
cmake ..
make
cd bin
./djiosdk-flightcontrol-sample UserConfig.txt UserConfig.txt
#./djiosdk-telemetry-sample UserConfig.txt UserConfig.txt

