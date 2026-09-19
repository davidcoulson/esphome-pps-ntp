#!/bin/sh
# Builds and runs the host-side simulation. Pass any argument to see the component's log output.
set -e
cd "$(dirname "$0")"
c++ -std=c++20 -O1 -Wall -Wno-unused -I stubs -include unistd.h sim.cpp ../components/pps_ntp/pps_ntp.cpp -o /tmp/pps_ntp_sim
/tmp/pps_ntp_sim "$@"
