#!/bin/sh
# Host-side unit tests for the AI code (no robot needed).
# Run from anywhere on Linux, macOS, or WSL:   sh tests/host/run_tests.sh
# Needs g++ or clang++ (set CXX=clang++ to use clang).
set -e
cd "$(dirname "$0")/../.."
CXX=${CXX:-g++}
mkdir -p build
for id in 1 2; do
  $CXX -std=gnu++11 -fno-exceptions -fno-rtti -fshort-enums -w \
    -Dprivate=public -Dprotected=public -DVexV5 -DROBOT_ID=$id \
    -Iinclude -Itests/host/fake_vex \
    src/JAR-Template/*.cpp src/autons.cpp src/robot-config.cpp src/ai/*.cpp \
    tests/host/test_ai.cpp -o build/host_tests_$id
  echo "ROBOT_ID=$id:"
  ./build/host_tests_$id
done
