#!/bin/sh
# Host regression tests: real orchestrator, DLMS parser and (in the RF suite)
# real CRC/AES-GCM; ESPHome runtime and the radio are replaced by stubs.
set -eu
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT
CXXFLAGS="-std=c++17 -Wall -Wextra -Wno-unused-parameter -fsanitize=address,undefined -fno-omit-frame-pointer -g"
INC="-Itests/stubs -Icomponents/nartis_rf_meter"
export ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1

"${CXX:-g++}" $CXXFLAGS $INC \
  components/nartis_rf_meter/dlms_client.cpp \
  components/nartis_rf_meter/nartis_rf_meter.cpp tests/test_transactions.cpp \
  -o "$build_dir/test_transactions"
"$build_dir/test_transactions"

"${CXX:-g++}" $CXXFLAGS $INC \
  components/nartis_rf_meter/dlms_client.cpp components/nartis_rf_meter/rf_data.cpp \
  components/nartis_rf_meter/nartis_rf_meter.cpp tests/test_rf_integration.cpp \
  -o "$build_dir/test_rf_integration"
"$build_dir/test_rf_integration"

"${CXX:-g++}" $CXXFLAGS $INC \
  components/nartis_rf_meter/dlms_client.cpp tests/test_dlms_parser.cpp \
  -o "$build_dir/test_dlms_parser"
"$build_dir/test_dlms_parser"
